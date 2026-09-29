package config.neds;

import artery.application.ItsG5Service;

//
// Confidence of received CAM and CPM content: Paper 1 (plausibility of every claim track) plus anchors in
// sources the author does not control (the receiver's radar; independent authors on the same LDM track).
// The error models (GNSS, radar, field of view) are read from this station's SensorNoiseService.
//
simple CollectiveTrustService like ItsG5Service
{
    parameters:
        @class(artery::CollectiveTrustService);
        @display("i=block/app2");

        // ── Components (ablation) ──
        bool enableRadar = default(true);             // the receiver's radar anchors claims
        bool enablePeers = default(true);             // independent authors confirm claims the radar cannot judge
        bool enablePoseCheck = default(true);         // CPM reference pose vs the author's CAM track
        bool enableAdmissibility = default(true);     // object reports that cannot be physical
        bool enableObjectKinematics = default(true);  // Paper 1 on perceived-object tracks
        bool enableSilence = default(true);           // two covering authors that report nothing contradict an
                                                      // unanchored CAM claim (at most one test per second and sender)

        // ── Plausibility: Paper 1, unchanged (values fitted in Paper 1 on VeReMi Extension) ──
        string drAlgorithm = default("HistoryAwareCTRA");
        int historySize = default(4);
        double maxPositionDeviation @unit(m) = default(30.85m);
        double contextSpeedWeight = default(0.9874);
        double contextYawWeight = default(0.0387);
        double speedTolerance = default(4.0);          // [m/s] implied vs reported speed
        double trackTimeout @unit(s) = default(5s);
        double defaultVehicleLength @unit(m) = default(5m);

        // ── Anchoring (Paper 1's counterless filter, fed by independent sources) ──
        double anchorSupport = default(1.0);           // own radar confirms (Paper 1's reward)
        double peerSupport = default(0.5);             // per independent author, at most anchorSupport
        double anchorPenalty = default(4.0);           // own radar contradicts: two contradictions flag an anchored claim
        double inclusionPeriod @unit(s) = default(1s); // honest authors re-report tracked objects at least this often

        // ── States of a message ──
        double implausibleBelow = default(0.5);        // on P (Paper 1)
        double contradictedBelow = default(0.4);       // on A
        double anchoredFrom = default(0.58);           // on A: three consecutive confirmations

        // ── Object admissibility ──
        double maxObjectSpeed = default(70.0);         // [m/s]
        double rangeMargin = default(1.2);             // objects beyond max sensor range * margin are inadmissible

        // ── Association (greedy gated assignment, CTRA projection) and gating; position errors come from the error models ──
        double maxSyncDt @unit(s) = default(1.1s);         // CAMs may be 1 s apart (ETSI EN 302 637-2)
        double kappaGate = default(3.0);
        double lambdaV = default(1.0);
        double sigmaVel = default(2.0);                    // [m/s] speed tolerance of the assignment cost
        double aMax = default(4.0);                        // [m/s^2] unmodelled acceleration over the projection
        double occluderRadius @unit(m) = default(1.2m);
        bool egoGnssError = default(true);                 // the receiver places its view with its own GNSS fix
        bool buildingOcclusion = default(true);
        string cpmEncoding = default("artery");            // "artery" (CpService quirks) or "etsi"

        // ── Metrics and diagnostics (evaluation only) ──
        double metricsInterval @unit(s) = default(1s);     // claim file: one row per claim track and interval
        double metricsWarmup @unit(s) = default(2s);
        bool visualizeTrust = default(false);              // colour vehicles in sumo-gui by their CAM state
        string claimRecordFile = default("");              // CSV of attributed confidences (sampled)
}
