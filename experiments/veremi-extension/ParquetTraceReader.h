#pragma once

#include "ReceivedVector.h"
#include "MotionVectorAdapter.h"

#include <cmath>

#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/reader.h>

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

/**
 * @brief Parser for the VeReMi Extension columnar traces (.parquet).
 *
 * Decodes straight into the engine's motion-vector model. 
 * These traces are split one file per receiver and carry no receiver column, so
 * the receiver id is recovered from the filename.
 */
class ParquetTraceReader {
public:
    /// Recover the receiver id from a traceJSON-<receiver>-<...>.parquet name.
    static long extractReceiverIdFromPath(const std::string& fpath)
    {
        std::string fname = std::filesystem::path(fpath).filename().string();
        size_t p1 = fname.find('-');
        if (p1 != std::string::npos) {
            size_t p2 = fname.find('-', p1 + 1);
            if (p2 != std::string::npos) {
                try {
                    return std::stol(fname.substr(p1 + 1, p2 - p1 - 1));
                } catch (...) {}
            }
        }
        return -1;
    }

    /// All per-receiver trace files in a scenario directory, in order.
    static std::vector<std::string> getTraceFiles(const std::string& dirPath)
    {
        std::vector<std::string> files;
        if (!std::filesystem::exists(dirPath) || !std::filesystem::is_directory(dirPath)) {
            return files;
        }
        for (const auto& entry : std::filesystem::directory_iterator(dirPath)) {
            if (entry.path().extension() == ".parquet" &&
                entry.path().filename().string().find("traceJSON") == 0) {
                files.push_back(entry.path().string());
            }
        }
        std::sort(files.begin(), files.end());
        return files;
    }

    /// Decode one trace file. Only received beacons (type == 3) are returned.
    static std::vector<ReceivedVector> read(const std::string& fpath)
    {
        const long receiverId = extractReceiverIdFromPath(fpath);
        std::vector<ReceivedVector> records;

        auto infile_result = arrow::io::ReadableFile::Open(fpath);
        if (!infile_result.ok()) return records;
        auto infile = infile_result.ValueOrDie();

        auto reader_result = parquet::arrow::OpenFile(infile, arrow::default_memory_pool());
        if (!reader_result.ok()) return records;
        auto reader = std::move(reader_result.ValueOrDie());

        auto table_result = reader->ReadTable();
        if (!table_result.ok()) return records;
        auto table = table_result.ValueOrDie();

        auto type_col    = table->GetColumnByName("type");
        auto rcvTime_col = table->GetColumnByName("rcvTime");
        auto sender_col  = table->GetColumnByName("sender");
        auto pos_x_col   = table->GetColumnByName("pos_x");
        auto pos_y_col   = table->GetColumnByName("pos_y");
        auto spd_x_col   = table->GetColumnByName("spd_x");
        auto spd_y_col   = table->GetColumnByName("spd_y");
        auto hed_x_col   = table->GetColumnByName("hed_x");
        auto hed_y_col   = table->GetColumnByName("hed_y");
        auto acl_x_col   = table->GetColumnByName("acl_x");
        auto acl_y_col   = table->GetColumnByName("acl_y");

        if (!type_col || !rcvTime_col || !sender_col || !pos_x_col || !pos_y_col ||
            !spd_x_col || !spd_y_col || !hed_x_col || !hed_y_col) {
            return records;
        }

        for (int chunk = 0; chunk < type_col->num_chunks(); ++chunk) {
            auto type_arr   = std::static_pointer_cast<arrow::Int64Array>(type_col->chunk(chunk));
            auto time_arr   = std::static_pointer_cast<arrow::DoubleArray>(rcvTime_col->chunk(chunk));
            auto sender_arr = std::static_pointer_cast<arrow::Int64Array>(sender_col->chunk(chunk));
            auto px_arr     = std::static_pointer_cast<arrow::DoubleArray>(pos_x_col->chunk(chunk));
            auto py_arr     = std::static_pointer_cast<arrow::DoubleArray>(pos_y_col->chunk(chunk));
            auto sx_arr     = std::static_pointer_cast<arrow::DoubleArray>(spd_x_col->chunk(chunk));
            auto sy_arr     = std::static_pointer_cast<arrow::DoubleArray>(spd_y_col->chunk(chunk));
            auto hx_arr     = std::static_pointer_cast<arrow::DoubleArray>(hed_x_col->chunk(chunk));
            auto hy_arr     = std::static_pointer_cast<arrow::DoubleArray>(hed_y_col->chunk(chunk));

            std::shared_ptr<arrow::DoubleArray> ax_arr = nullptr;
            std::shared_ptr<arrow::DoubleArray> ay_arr = nullptr;
            if (acl_x_col && acl_y_col &&
                chunk < acl_x_col->num_chunks() && chunk < acl_y_col->num_chunks()) {
                ax_arr = std::static_pointer_cast<arrow::DoubleArray>(acl_x_col->chunk(chunk));
                ay_arr = std::static_pointer_cast<arrow::DoubleArray>(acl_y_col->chunk(chunk));
            }

            for (int64_t i = 0; i < type_arr->length(); ++i) {
                if (type_arr->IsNull(i) || type_arr->Value(i) != 3) continue;

                ReceivedVector rv;
                rv.receiver = receiverId;
                rv.sender   = sender_arr->Value(i);

                rv.mv.time = time_arr->Value(i);
                rv.mv.lon  = px_arr->Value(i);
                rv.mv.lat  = py_arr->Value(i);

                const double sx = sx_arr->Value(i);
                const double sy = sy_arr->Value(i);
                rv.mv.speed = std::sqrt(sx * sx + sy * sy);
                rv.mv.heading = MotionVectorAdapter::headingDegrees(hx_arr->Value(i),
                                                                   hy_arr->Value(i));

                // REVIEW: payload-reported acceleration, projected onto the heading.
                // This trusts the sender; the engine regresses a and omega over the
                // tracking window instead (paper Eq. 1 and 2, "payload trust
                // independence", Sec. V-F), so this only seeds the short-window
                // fallback. Yaw rate is not logged in VeReMi traces at all.
                if (ax_arr && ay_arr && !ax_arr->IsNull(i) && !ay_arr->IsNull(i)) {
                    const double hRad = rv.mv.heading * M_PI / 180.0;
                    rv.mv.accel = ax_arr->Value(i) * std::sin(hRad)
                                + ay_arr->Value(i) * std::cos(hRad);
                }
                rv.mv.yawRate = 0.0;

                records.push_back(rv);
            }
        }

        std::sort(records.begin(), records.end());
        return records;
    }
};
