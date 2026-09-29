package config.neds;

import artery.application.ItsG5Service;

//
// Confidence of received CAM and CPM content: Paper 1 (plausibility of every claim track) plus corroboration by
// independent authors whose statements land on the same LDM track (it only confirms).
// The error models (GNSS, CPM object error) are read from this station's SensorNoiseService.
//
simple CollectiveTrustService like ItsG5Service
{
    parameters:
        @class(artery::CollectiveTrustService);
        @display("i=block/app2");

        // ── Components (ablation) ──
        bool enablePeers = default(true);             // independent authors reporting the same object confirm it
        bool enablePoseCheck = default(true);         // CPM reference pose vs the author's CAM track
        bool enableAdmissibility = default(true);     // object reports that cannot be physical
        bool enableObjectKinematics = default(true);  // Paper 1 on perceived-object tracks

        // ── Plausibility: Paper 1, unchanged (values fitted in Paper 1 on VeReMi Extension) ──
        string drAlgorithm = default("HistoryAwareCTRA");
        int historySize = default(4);
        double maxPositionDeviation @unit(m) = default(30.85m);
        double contextSpeedWeight = default(0.9874);
        double contextYawWeight = default(0.0387);
        double speedTolerance = default(4.0);          // [m/s] implied vs reported speed
        double trackTimeout @unit(s) = default(5s);
        double defaultVehicleLength @unit(m) = default(5m);

        // ── Corroboration (Paper 1's counterless filter, fed only by independent authors) ──
        double maxSupport = default(1.0);              // Paper 1's reward: the most one message can gain
        double peerSupport = default(0.5);             // per independent author, at most maxSupport
        double inclusionPeriod @unit(s) = default(1s); // window in which other authors' reports of the same object confirm

        // ── States of a message ──
        double implausibleBelow = default(0.5);        // on P (Paper 1)
        double corroboratedFrom = default(0.58);       // on C: three consecutive full confirmations

        // ── Object admissibility ──
        double maxObjectSpeed = default(70.0);         // [m/s]
        double maxObjectRange = default(120.0);        // [m] authors' perception range (100 m, ITU-R M.2057-1 Radar B) + 20 %

        // ── Association (greedy gated assignment, CTRA projection) and gating; position errors come from the error models ──
        double maxSyncDt @unit(s) = default(1.1s);         // CAMs may be 1 s apart (ETSI EN 302 637-2)
        double kappaGate = default(3.0);
        double lambdaV = default(1.0);
        double sigmaVel = default(2.0);                    // [m/s] speed tolerance of the assignment cost
        double aMax = default(4.0);                        // [m/s^2] unmodelled acceleration over the projection
        string cpmEncoding = default("artery");            // "artery" (CpService quirks) or "etsi"

        // ── Metrics and diagnostics (evaluation only) ──
        double metricsInterval @unit(s) = default(1s);     // claim file: one row per claim track and interval
        double metricsWarmup @unit(s) = default(2s);
        bool visualizeTrust = default(false);              // colour vehicles in sumo-gui by their CAM state
        string claimRecordFile = default("");              // CSV of attributed confidences (sampled)
}
