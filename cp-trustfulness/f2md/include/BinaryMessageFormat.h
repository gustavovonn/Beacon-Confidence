#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <filesystem>

#include "BasicSafetyMessage.h"
#include "IDeadReckoningModel.h"

#pragma pack(push, 1)
struct SerializedBsmRecord {
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

static const uint64_t V2X_BSM_MAGIC = 0x56325842534D3031ULL; // "V2XBSM01"

struct RawMessage {
    double rcvTime;
    long sender;
    double pos_x;
    double pos_y;
    double speed;
    double heading;
    double accel;     // Computed during extraction
    double yawRate;   // Computed during extraction
    
    bool operator<(const RawMessage& other) const {
        return rcvTime < other.rcvTime;
    }
};

class BinaryMessageExtractor {
public:
    static bool isBinaryFile(const std::string& path) {
        if (!std::filesystem::exists(path) || std::filesystem::is_directory(path)) {
            return false;
        }
        std::ifstream fin(path, std::ios::binary);
        if (!fin.is_open()) return false;
        uint64_t magic = 0;
        fin.read(reinterpret_cast<char*>(&magic), sizeof(magic));
        return (magic == V2X_BSM_MAGIC);
    }

    static std::vector<SerializedBsmRecord> readRawRecords(const std::string& path) {
        std::vector<std::string> files;
        if (std::filesystem::is_directory(path)) {
            for (const auto& entry : std::filesystem::directory_iterator(path)) {
                if (entry.path().extension() == ".bin" || entry.path().extension() == ".dat") {
                    files.push_back(entry.path().string());
                }
            }
        } else if (std::filesystem::exists(path)) {
            files.push_back(path);
        }
        std::sort(files.begin(), files.end());

        std::vector<SerializedBsmRecord> all_records;

        for (const auto& fpath : files) {
            std::ifstream fin(fpath, std::ios::binary);
            if (!fin.is_open()) continue;

            uint64_t magic = 0;
            fin.read(reinterpret_cast<char*>(&magic), sizeof(magic));
            if (magic != V2X_BSM_MAGIC) {
                std::cerr << "[!] Warning: Invalid magic header in " << fpath << "\n";
                continue;
            }

            uint64_t count = 0;
            fin.read(reinterpret_cast<char*>(&count), sizeof(count));
            if (count == 0) continue;

            size_t prev_size = all_records.size();
            all_records.resize(prev_size + count);
            fin.read(reinterpret_cast<char*>(&all_records[prev_size]), count * sizeof(SerializedBsmRecord));
        }

        std::sort(all_records.begin(), all_records.end(), [](const SerializedBsmRecord& a, const SerializedBsmRecord& b) {
            return a.arrivalTime < b.arrivalTime;
        });

        return all_records;
    }

    static std::vector<RawMessage> extractRawMessages(const std::string& path) {
        auto records = readRawRecords(path);
        std::vector<RawMessage> messages;
        messages.reserve(records.size());

        for (const auto& rec : records) {
            RawMessage msg;
            msg.rcvTime = rec.arrivalTime;
            msg.sender = rec.sender;
            msg.pos_x = rec.px;
            msg.pos_y = rec.py;
            msg.speed = std::sqrt(rec.sx * rec.sx + rec.sy * rec.sy);

            double h_deg = std::atan2(rec.hx, rec.hy) * 180.0 / M_PI;
            if (h_deg < 0) h_deg += 360.0;
            msg.heading = h_deg;
            msg.accel = 0.0;
            msg.yawRate = 0.0;

            messages.push_back(msg);
        }
        return messages;
    }

    static std::vector<BasicSafetyMessage> extractF2mdMessages(const std::string& path) {
        auto records = readRawRecords(path);
        std::vector<BasicSafetyMessage> messages;
        messages.reserve(records.size());

        for (const auto& rec : records) {
            BasicSafetyMessage msg;
            msg.arrivalTime = rec.arrivalTime;
            msg.pseudonym = rec.sender;
            msg.receiver_id = rec.receiver_id;
            msg.receiver_pseudo = rec.receiver_pseudo;

            msg.pos = veins::Coord(rec.px, rec.py, 0);
            msg.posConf = veins::Coord(rec.pnx, rec.pny, 0);

            msg.speed = veins::Coord(rec.sx, rec.sy, 0);
            msg.speedConf = veins::Coord(rec.snx, rec.sny, 0);

            msg.accel = veins::Coord(rec.ax, rec.ay, 0);
            msg.accelConf = veins::Coord(rec.anx, rec.any, 0);

            msg.heading = veins::Coord(rec.hx, rec.hy, 0);
            msg.headingConf = veins::Coord(rec.hnx, rec.hny, 0);

            msg.recv_pos = veins::Coord(rec.epx, rec.epy, 0);
            msg.recv_heading = veins::Coord(rec.ehx, rec.ehy, 0);
            msg.recv_speed = veins::Coord(rec.esx, rec.esy, 0);

            messages.push_back(msg);
        }
        return messages;
    }
};
