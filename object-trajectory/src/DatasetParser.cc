#include "../include/DatasetParser.h"
#include "../shared/Geodesy.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>
#include <sstream>

namespace ot {

namespace {

/// Split one CSV line. The generator writes no quoted fields, so a plain split is correct here --
/// and a parser that pretends to handle quoting it never sees is a parser nobody has tested.
std::vector<std::string> split(const std::string& line)
{
    std::vector<std::string> out;
    std::string cell;
    std::istringstream is(line);
    while (std::getline(is, cell, ',')) {
        out.push_back(cell);
    }
    // a trailing comma means a final empty field, which getline does not produce
    if (!line.empty() && line.back() == ',') {
        out.emplace_back();
    }
    return out;
}

double toDouble(const std::string& s, double fallback = 0.0)
{
    if (s.empty()) {
        return fallback;
    }
    try {
        return std::stod(s);
    } catch (...) {
        return fallback;
    }
}

long long toLong(const std::string& s, long long fallback = 0)
{
    if (s.empty()) {
        return fallback;
    }
    try {
        return std::stoll(s);
    } catch (...) {
        return fallback;
    }
}

/// Column index by name, so a reordered or extended CSV does not silently shift every field.
struct Header {
    std::map<std::string, std::size_t> index;

    bool read(std::ifstream& in)
    {
        std::string line;
        if (!std::getline(in, line)) {
            return false;
        }
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        const std::vector<std::string> cols = split(line);
        for (std::size_t i = 0; i < cols.size(); ++i) {
            index[cols[i]] = i;
        }
        return true;
    }

    bool has(std::initializer_list<const char*> names, std::string& missing) const
    {
        for (const char* n : names) {
            if (index.find(n) == index.end()) {
                missing = n;
                return false;
            }
        }
        return true;
    }

    /// Empty string when the row is short, which a trailing optional column legitimately is.
    const std::string& get(const std::vector<std::string>& row, const char* name) const
    {
        static const std::string empty;
        const auto it = index.find(name);
        if (it == index.end() || it->second >= row.size()) {
            return empty;
        }
        return row[it->second];
    }
};

constexpr double kDegToRad = M_PI / 180.0;

}  // namespace

bool DatasetParser::load(const std::string& dir, std::string& error)
{
    m_deliveries.clear();
    m_info = DatasetInfo{};
    m_info.path = dir;

    // ---- CAMs ------------------------------------------------------------------------------
    std::ifstream camFile(dir + "/cam.csv");
    if (!camFile) {
        error = "cannot open " + dir + "/cam.csv";
        return false;
    }
    Header camHdr;
    std::string missing;
    if (!camHdr.read(camFile)) {
        error = "cam.csv has no header row (the file is empty or still being written)";
        return false;
    }
    if (!camHdr.has({"message_id", "t", "sender", "lat", "lon", "speed", "heading"}, missing)) {
        error = "cam.csv: missing column '" + missing + "'";
        return false;
    }

    // The tangent-plane origin is the first CAM position in the file. Any fixed point would do;
    // what matters is that it is derived from the data and recorded, so two parses of the same
    // dataset place every object at exactly the same metric coordinates.
    bool haveOrigin = false;
    geo::GeoPoint origin;

    struct Content {
        std::vector<MotionVector> vectors;
    };
    std::map<std::string, Content> byMessage;

    std::string line;
    while (std::getline(camFile, line)) {
        if (line.empty()) {
            continue;
        }
        const std::vector<std::string> row = split(line);
        const std::string id = camHdr.get(row, "message_id");
        if (id.empty()) {
            continue;
        }
        const double lat = toDouble(camHdr.get(row, "lat"));
        const double lon = toDouble(camHdr.get(row, "lon"));
        if (!haveOrigin) {
            origin = geo::GeoPoint{lat, lon};
            haveOrigin = true;
        }
        const geo::LocalPoint p = geo::toLocal(origin, geo::GeoPoint{lat, lon});

        MotionVector v;
        v.t = toDouble(camHdr.get(row, "t"));
        v.x = p.east;
        v.y = p.north;
        v.speed = toDouble(camHdr.get(row, "speed"));
        v.heading = toDouble(camHdr.get(row, "heading")) * kDegToRad;
        const std::string yaw = camHdr.get(row, "yaw_rate");
        v.yawRate = toDouble(yaw);
        v.hasYawRate = !yaw.empty();
        v.accel = toDouble(camHdr.get(row, "accel"));
        v.length = toDouble(camHdr.get(row, "length"));
        v.objectClass = 1;   // a CAM sender is a vehicle
        v.origin = Origin::Cam;
        v.author = static_cast<uint64_t>(toLong(camHdr.get(row, "sender")));
        // A CAM is self-reported, so the oracle is the sender itself -- no lookup needed, and it is
        // what makes CAM and CPM content scorable on the same footing.
        v.trueStation = static_cast<int64_t>(v.author);
        v.hasVelocity = true;

        byMessage[id].vectors.push_back(v);
        ++m_info.cams;
    }

    if (!haveOrigin) {
        error = "cam.csv holds no rows: cannot fix a tangent-plane origin";
        return false;
    }
    m_info.originLat = origin.lat;
    m_info.originLon = origin.lon;

    // ---- CPM objects -------------------------------------------------------------------------
    std::ifstream cpmFile(dir + "/cpm.csv");
    if (!cpmFile) {
        error = "cannot open " + dir + "/cpm.csv";
        return false;
    }
    Header cpmHdr;
    if (!cpmHdr.read(cpmFile)) {
        error = "cpm.csv has no header row (the file is empty or still being written)";
        return false;
    }
    if (!cpmHdr.has({"message_id", "t", "author", "author_lat", "author_lon", "author_heading",
                     "object_id", "dx", "dy", "speed", "heading"},
                    missing)) {
        error = "cpm.csv: missing column '" + missing + "'";
        return false;
    }
    std::set<std::string> cpmIds;

    while (std::getline(cpmFile, line)) {
        if (line.empty()) {
            continue;
        }
        const std::vector<std::string> row = split(line);
        const std::string id = cpmHdr.get(row, "message_id");
        if (id.empty()) {
            continue;
        }
        cpmIds.insert(id);

        // Compose: author pose, then the author-relative offset. The dataset deliberately keeps
        // these apart so an injector can perturb the author's pose and the sensor reading
        // independently; placing the object is the parser's job, not the recording's.
        const geo::LocalPoint authorXY =
            geo::toLocal(origin, geo::GeoPoint{toDouble(cpmHdr.get(row, "author_lat")),
                                               toDouble(cpmHdr.get(row, "author_lon"))});

        MotionVector v;
        v.t = toDouble(cpmHdr.get(row, "t"));
        v.x = authorXY.east + toDouble(cpmHdr.get(row, "dx"));
        v.y = authorXY.north + toDouble(cpmHdr.get(row, "dy"));
        v.speed = toDouble(cpmHdr.get(row, "speed"));
        v.heading = toDouble(cpmHdr.get(row, "heading")) * kDegToRad;
        v.hasYawRate = false;
        v.objectClass = 1;
        v.origin = Origin::CpmObject;
        v.author = static_cast<uint64_t>(toLong(cpmHdr.get(row, "author")));
        v.objectId = static_cast<uint32_t>(toLong(cpmHdr.get(row, "object_id")));
        v.hasVelocity = toLong(cpmHdr.get(row, "has_velocity"), 1) != 0;

        // Empty means the oracle did not resolve this object, which is not the same as station 0.
        const std::string truth = cpmHdr.get(row, "true_station");
        v.trueStation = truth.empty() ? -1 : static_cast<int64_t>(toLong(truth));
        if (!truth.empty()) {
            ++m_info.objectsWithOracle;
        }

        byMessage[id].vectors.push_back(v);
        ++m_info.objects;
    }
    m_info.cpms = cpmIds.size();

    // ---- receptions --------------------------------------------------------------------------
    std::ifstream rxFile(dir + "/reception.csv");
    if (!rxFile) {
        error = "cannot open " + dir + "/reception.csv";
        return false;
    }
    Header rxHdr;
    if (!rxHdr.read(rxFile)) {
        error = "reception.csv has no header row (the file is empty or still being written)";
        return false;
    }
    if (!rxHdr.has({"message_id", "receiver", "t_rx"}, missing)) {
        error = "reception.csv: missing column '" + missing + "'";
        return false;
    }

    while (std::getline(rxFile, line)) {
        if (line.empty()) {
            continue;
        }
        const std::vector<std::string> row = split(line);
        const std::string id = rxHdr.get(row, "message_id");
        ++m_info.receptions;

        const auto it = byMessage.find(id);
        if (it == byMessage.end() || it->second.vectors.empty()) {
            // A reception whose content is absent: the message was received before the warmup cut,
            // or it carried no perceived object. Nothing to feed the engine, so it is dropped here
            // rather than becoming an empty call that would still age every track.
            continue;
        }

        Delivery d;
        d.messageId = id;
        d.receiver = static_cast<uint64_t>(toLong(rxHdr.get(row, "receiver")));
        d.tRx = toDouble(rxHdr.get(row, "t_rx"));
        d.vectors = it->second.vectors;
        m_deliveries.push_back(std::move(d));
    }

    // Arrival order, with receiver as a tie-break so a run is reproducible rather than dependent on
    // the order reception.csv happened to be written in.
    std::sort(m_deliveries.begin(), m_deliveries.end(), [](const Delivery& a, const Delivery& b) {
        if (a.tRx != b.tRx) {
            return a.tRx < b.tRx;
        }
        if (a.receiver != b.receiver) {
            return a.receiver < b.receiver;
        }
        return a.messageId < b.messageId;
    });

    m_info.deliveries = m_deliveries.size();
    if (!m_deliveries.empty()) {
        m_info.firstTime = m_deliveries.front().tRx;
        m_info.lastTime = m_deliveries.back().tRx;
    }
    return true;
}

std::vector<const Delivery*> DatasetParser::forReceiver(uint64_t receiver) const
{
    std::vector<const Delivery*> out;
    for (const Delivery& d : m_deliveries) {
        if (d.receiver == receiver) {
            out.push_back(&d);
        }
    }
    return out;   // already in arrival order: m_deliveries is sorted
}

std::vector<uint64_t> DatasetParser::receivers() const
{
    std::set<uint64_t> seen;
    for (const Delivery& d : m_deliveries) {
        seen.insert(d.receiver);
    }
    return {seen.begin(), seen.end()};
}

}  // namespace ot
