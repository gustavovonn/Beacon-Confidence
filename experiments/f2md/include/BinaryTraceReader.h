#ifndef BINARY_TRACE_READER_H_
#define BINARY_TRACE_READER_H_

#include "BsmRecord.h"
#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

/**
 * @brief Parser for the zero-dependency packed binary trace format (.bin).
 *
 * Layout: an 8-byte magic ("V2XBSM01"), an 8-byte record count, then that many
 * fixed-size packed records.
 */
class BinaryTraceReader {
public:
    static constexpr uint64_t MAGIC = 0x56325842534D3031ULL; // "V2XBSM01"

#pragma pack(push, 1)
    /// On-disk record. Field order and packing must match the exporter exactly.
    struct SerializedRecord {
        double arrivalTime;
        int64_t sender;
        int64_t receiver_id;
        int64_t receiver_pseudo;
        double px, py;
        double pnx, pny;
        double sx, sy;
        double snx, sny;
        double ax, ay;
        double anx, any;
        double hx, hy;
        double hnx, hny;
        double epx, epy;
        double ehx, ehy;
        double esx, esy;
    };
#pragma pack(pop)

    /// True if the file begins with the expected magic header.
    static bool isBinaryTrace(const std::string& path)
    {
        if (!std::filesystem::exists(path) || std::filesystem::is_directory(path)) {
            return false;
        }
        std::ifstream fin(path, std::ios::binary);
        if (!fin.is_open()) return false;
        uint64_t magic = 0;
        fin.read(reinterpret_cast<char*>(&magic), sizeof(magic));
        return (magic == MAGIC);
    }

    /// True if the path is, or is a directory containing, a binary trace.
    static bool accepts(const std::string& path)
    {
        if (isBinaryTrace(path) || path.ends_with(".bin") || path.ends_with(".dat")) {
            return true;
        }
        if (std::filesystem::is_directory(path)) {
            for (const auto& entry : std::filesystem::directory_iterator(path)) {
                const auto ext = entry.path().extension();
                if (ext == ".bin" || ext == ".dat") return true;
            }
        }
        return false;
    }

    /// Read one trace file, or every trace in a directory, sorted by arrival time.
    static std::vector<BsmRecord> read(const std::string& path)
    {
        std::vector<std::string> files;
        if (std::filesystem::is_directory(path)) {
            for (const auto& entry : std::filesystem::directory_iterator(path)) {
                const auto ext = entry.path().extension();
                if (ext == ".bin" || ext == ".dat") {
                    files.push_back(entry.path().string());
                }
            }
        } else if (std::filesystem::exists(path)) {
            files.push_back(path);
        }
        std::sort(files.begin(), files.end());

        std::vector<SerializedRecord> packed;
        for (const auto& fpath : files) {
            std::ifstream fin(fpath, std::ios::binary);
            if (!fin.is_open()) continue;

            uint64_t magic = 0;
            fin.read(reinterpret_cast<char*>(&magic), sizeof(magic));
            if (magic != MAGIC) {
                std::cerr << "[!] Warning: Invalid magic header in " << fpath << "\n";
                continue;
            }

            uint64_t count = 0;
            fin.read(reinterpret_cast<char*>(&count), sizeof(count));
            if (count == 0) continue;

            const size_t prev = packed.size();
            packed.resize(prev + count);
            fin.read(reinterpret_cast<char*>(&packed[prev]), count * sizeof(SerializedRecord));
        }

        std::sort(packed.begin(), packed.end(),
                  [](const SerializedRecord& a, const SerializedRecord& b) {
                      return a.arrivalTime < b.arrivalTime;
                  });

        std::vector<BsmRecord> records;
        records.reserve(packed.size());
        for (const auto& p : packed) {
            BsmRecord r;
            r.arrivalTime    = p.arrivalTime;
            r.sender         = p.sender;
            r.receiverId     = p.receiver_id;
            r.receiverPseudo = p.receiver_pseudo;
            r.px  = p.px;  r.py  = p.py;
            r.pnx = p.pnx; r.pny = p.pny;
            r.sx  = p.sx;  r.sy  = p.sy;
            r.snx = p.snx; r.sny = p.sny;
            r.ax  = p.ax;  r.ay  = p.ay;
            r.anx = p.anx; r.any = p.any;
            r.hx  = p.hx;  r.hy  = p.hy;
            r.hnx = p.hnx; r.hny = p.hny;
            r.epx = p.epx; r.epy = p.epy;
            r.ehx = p.ehx; r.ehy = p.ehy;
            r.esx = p.esx; r.esy = p.esy;
            records.push_back(r);
        }
        return records;
    }
};

#endif // BINARY_TRACE_READER_H_
