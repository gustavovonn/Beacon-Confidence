package config.neds;

import artery.application.ItsG5Service;

//
// Confidence of received CAM and CPM motion vectors: one confidence per claim stream (a sender's trajectory, or one
// author's reports of one object), fed by Paper 1's checks and by corroboration from other stations whose admitted
// vectors land on the same LDM track (evidence in favour only).
// The error models (GNSS, CPM object error) are read from this station's SensorNoiseService.
//
simple CollectiveTrustService like ItsG5Service
{
    parameters:
        @class(artery::CollectiveTrustService);
        @display("i=block/app2");

        // ── Components (ablation) ──
        bool enableCpm = default(true);               // ablation: receiver ignores CPMs (traffic unchanged)
        bool enablePeers = default(true);             // corroboration by other stations on the same LDM track
        bool enablePoseCheck = default(true);         // CPM reference pose vs the author's CAM track
        bool enableAdmissibility = default(true);     // impossible vectors: too fast, out of range, inside the author
        bool enableObjectKinematics = default(true);  // Paper 1 on perceived-object tracks

        // ── Plausibility: Paper 1, unchanged (values fitted in Paper 1 on VeReMi Extension) ──
        string drAlgorithm = default("HistoryAwareCTRA");
        int historySize = default(4);
        // d_max per claim kind. scripts/context_tuning.py refitted both over 4.5M honest vectors
        // (3 seeds x 60 s, LuST); the intercepts came out 5.2x apart, and a pooled fit is invalid because
        // PerceivedObject carries no yaw rate, so the yaw regressor degenerates into a CAM/object indicator
        // and its coefficient turns negative. Only the OBJECT fit is adopted:
        //
        //   CAM     keeps Paper 1's published values. Its refit (2.94 / 0.0789 / 0.0073) made trajectory
        //           consistency fire for the first time, but on the wrong population: 24.9 % of honest CAM
        //           flags against 7.0 % of falsified ones, costing +0.6 pp of honest FPR (1.26 -> 1.86 %)
        //           for a flat F1 (-0.02 .. +0.02 over seven profiles). Measured in exp_20261008_204623_dmax.
        //   object  takes the refit. There the same check is selective the other way round: 0.8-2.0 % of
        //           honest flags against 5.9-7.3 % of falsified, and removing it costs 0.57 F1.
        //
        // So the inherited threshold stays where it is inherited, and only the contributed kind is retuned.
        double maxPositionDeviation @unit(m) = default(30.85m);      // CAM streams, Paper 1
        double contextSpeedWeight = default(0.9874);
        double contextYawWeight = default(0.0387);
        double objectMaxPositionDeviation @unit(m) = default(15.38m);  // perceived-object streams, refitted
        double objectContextSpeedWeight = default(0.0517);
        double objectContextYawWeight = default(0.0);                  // unfittable: no yaw rate on the wire
        double speedTolerance = default(4.0);          // [m/s] implied vs reported speed
        double minReportInterval @unit(s) = default(0.001s);    // below this it is not a new report: dropped
        double minSpeedCheckInterval @unit(s) = default(0.1s);  // below this phase 0 is skipped; the vector
                                                                // still runs phase 1 and is appended.
                                                                // T_GenCamMin / T_GenCpmMin = 100 ms, valid
                                                                // only for a single-source stream
                                                             // skips VeReMi's repeated receiver logs; in a
                                                             // simulator it would silence phase 0 whenever a
                                                             // manoeuvring vehicle raises its CAM rate
        double trackTimeout @unit(s) = default(5s);
        double defaultVehicleLength @unit(m) = default(5m);

        // ── Corroboration: evidence in favour only, added to the same counterless update ──
        double maxSupport = default(1.0);              // Paper 1's reward: the most one vector can gain
        double peerSupport = default(0.5);            // per confirming station, at most maxSupport
        double inclusionPeriod @unit(s) = default(1s); // window in which other authors' reports of the same object confirm

        // ── States of a message ──
        double implausibleBelow = default(0.5);        // on P (Paper 1)

        // ── Impossible vectors ──
        double maxSpeed = default(70.0);               // [m/s] CAM and object vectors
        double maxObjectRange = default(120.0);        // [m] authors' perception range (100 m, ITU-R M.2057-1 Radar B) + 20 %

        // ── Association (gated global assignment, CTRA projection) and gating; position errors come from the error models ──
        string associationEngine = default("hungarian");   // "hungarian" (production) or "greedy" (ablation)
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
        string contextRecordFile = default("");            // CSV of phase 0/1 inputs, for d_max tuning only
}
