#ifndef COLLECTIVE_TRUST_SERVICE_H_
#define COLLECTIVE_TRUST_SERVICE_H_

#include "../../include/TrustEngine.h"
#include "ArteryAdapters.h"
#include <artery/application/ItsG5BaseService.h>
#include <array>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>

namespace traci { class VehicleController; }

namespace artery
{

class IdentityRegistry;
class SensorNoiseService;
class Timer;
class VehicleDataProvider;

/**
 * @brief Artery adapter of the confidence engine.
 *
 * Parses received CAMs and CPMs into claims, builds the receiver's own pose (with the station's referenced GNSS
 * error from SensorNoiseService) and feeds both to TrustEngine. It also scores
 * every attributed confidence against simulator ground truth, which the engine never sees.
 */
class CollectiveTrustService : public ItsG5BaseService
{
public:
    ~CollectiveTrustService() override;
    void initialize() override;
    void finish() override;
    void trigger() override;
    void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*, omnetpp::cObject*) override;
    bool requiresListener() const override { return false; }

private:
    /// What the simulation knows about a station (never shown to the engine).
    struct GroundTruth {
        bool known = false;
        bool faulty = false;           ///< any fault or attack configured
        bool camFalsified = false;     ///< its CAMs are falsified from camFaultStart on
        double camFaultStart = 0.0;
        std::string traciId;
    };

    /// Per-message outcome counters: [kind][falsified] (honest receivers, after the warm-up).
    struct Outcomes {
        uint64_t n = 0;
        uint64_t flagged = 0;         ///< implausible (P < 0.5)
        uint64_t corroborated = 0;    ///< C >= corroboratedFrom
        uint64_t peerConfirmed = 0;   ///< independent authors reported the same object in this message's window
        std::array<uint64_t, 5> reason{};  ///< flagged messages by the component behind the flag (Reason)
    };

    TrustEngineParams readParams();
    void handleCam(omnetpp::cObject*);
    void handleCpm(omnetpp::cObject*);
    void onClaim(const ClaimResult&);
    const EgoView& egoView();
    MessageTiming timing() const;
    const GroundTruth& truthOf(StationId id);
    void recordAssociation(const CpmSnapshot&, const CpmOutcome&);
    void updateVisualization(StationId id, ClaimState state);

    std::unique_ptr<TrustEngine> m_engine;
    const traci::VehicleController* m_controller = nullptr;
    const VehicleDataProvider* m_vdp = nullptr;
    const Timer* m_timer = nullptr;
    SensorNoiseService* m_noise = nullptr;  ///< this station's own positioning error
    IdentityRegistry* m_identities = nullptr;

    omnetpp::simsignal_t m_camSignal;
    omnetpp::simsignal_t m_cpmSignal;

    CpmEncoding m_cpmEncoding = CpmEncoding::Artery;
    std::optional<GridFrame> m_grid;
    double m_defaultLength = 5.0;
    double m_metricsInterval = 1.0;
    double m_metricsWarmup = 2.0;
    bool m_visualize = false;
    std::string m_claimRecordFile;

    EgoView m_ego;
    double m_egoBuiltAt = -1.0;
    bool m_honestReceiver = true;

    // the message being processed (for labelling its claims)
    bool m_camFalsified = false;
    uint64_t m_cpmReferenceTai = 0;

    std::map<StationId, GroundTruth> m_truth;
    std::array<std::array<Outcomes, 2>, 2> m_outcomes{};  ///< [cam=0 / object=1][honest=0 / falsified=1]
    std::map<std::tuple<StationId, int, uint16_t>, double> m_lastRecorded;  ///< claim -> last row in the claim file

    // Association (resolution) metrics against ground truth
    struct {
        uint64_t tp = 0;  ///< resolved to the right station
        uint64_t fp = 0;  ///< resolved to a wrong station (or a ghost resolved)
        uint64_t fn = 0;  ///< left unresolved although the true station was known to the receiver
        uint64_t tn = 0;  ///< left unresolved and not resolvable
    } m_assoc;
    uint64_t m_camsReceived = 0;
    uint64_t m_cpmsReceived = 0;
    uint64_t m_objectsReceived = 0;
};

} // namespace artery

#endif // COLLECTIVE_TRUST_SERVICE_H_
