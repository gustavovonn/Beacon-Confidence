#ifndef OT_ARTERY_DATASET_GENERATOR_H_
#define OT_ARTERY_DATASET_GENERATOR_H_

/**
 * ArteryDatasetGenerator -- records an honest trace of CAM and CPM content, and nothing else.
 *
 *
 * THE ONE THING THAT MUST NOT BE GOT WRONG: perceived objects are stored as offsets relative to
 * their author, with the author's pose in separate columns. Never absolute. Radar noise applies in
 * the author's sensor frame; the author's GNSS error then displaces that whole frame. Flattening to
 * absolute at recording time conflates the two, and no later injector can pull them apart again.
 *
 * Reuse, which is deliberate and limited:
 *   - MessageParsers::parseCam / parseCpm (cp-trustfulness/artery/include/ArteryAdapters.h) -- the
 *     CPM parsing exception. Reimplementing ASN.1 unpacking would be pure risk with no new insight.
 *   - EvaluationTruth::objectStation -- the oracle: which real station a perceived object actually
 *     is. Without it there is no association ground truth, and therefore no metric worth reporting.
 *     It is populated SENDER-side by SensorNoiseService, which must be loaded (configured with
 *     gnssErrorModel = "none" and radarErrorModel = "none", so it adds no error and only records).
 *   - Geodesy -- the local tangent plane.
 *
 * The engine under test never links against this file, and this file never links against the engine.
 */

#include "../../cp-trustfulness/artery/include/ArteryAdapters.h"  // MessageTiming, GridFrame, parsers
#include <artery/application/ItsG5BaseService.h>
#include <cstdint>
#include <map>
#include <optional>
#include <string>

namespace traci { class VehicleController; }

namespace artery
{

class Timer;
class VehicleDataProvider;
struct GridFrame;

class ArteryDatasetGenerator : public ItsG5BaseService
{
public:
    void initialize() override;
    void finish() override;
    void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*,
                       omnetpp::cObject*) override;
    bool requiresListener() const override { return false; }

private:
    void handleCam(omnetpp::cObject* obj);
    void handleCpm(omnetpp::cObject* obj);
    void writeMetadata() const;

    /// Wall-clock and TAI for the parsers. TAI is what a CPM's referenceTime is expressed in.
    MessageTiming timing() const;

    /// The SUMO projection's north bearing and scale, measured once on first use via two TraCI
    /// calls. Artery encodes CPM angles against grid north, not true north, so without this every
    /// perceived-object offset is rotated by the local meridian convergence.
    const GridFrame& grid();

    omnetpp::simsignal_t m_camSignal = -1;
    omnetpp::simsignal_t m_cpmSignal = -1;

    const VehicleDataProvider* m_vdp = nullptr;
    const Timer* m_timer = nullptr;
    const traci::VehicleController* m_controller = nullptr;
    std::optional<GridFrame> m_grid;

    std::string m_outputDir;
    double m_warmup = 0.0;          ///< [s] discard the ramp-up, where vehicles are still entering
    double m_defaultLength = 4.5;   ///< [m] used when a CAM omits the vehicle length
    std::string m_cpmEncoding;

    /// Receptions seen by THIS vehicle. Per-vehicle, so it is a meaningful scalar result.
    uint64_t m_receptions = 0;
};

}  // namespace artery

#endif  // OT_ARTERY_DATASET_GENERATOR_H_
