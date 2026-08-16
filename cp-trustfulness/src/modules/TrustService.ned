package src.modules;

import artery.application.ItsG5Service;

//
// Modular Trust & Dead Reckoning Evaluation Service
//
simple TrustService like ItsG5Service
{
    parameters:
        @class(artery::TrustService);
        @display("i=block/app");

        // Algorithm strategy selectors
        string drAlgorithm = default("HistoryAwareCTRA");
        int historySize = default(5);                // Window size for history-aware models
        string trustAlgorithm = default("BetaReputation");

        // Threshold parameters
        double maxPositionDeviation @unit(m) = default(5m);
        
        // Context-Aware Dynamic Threshold parameters
        double contextSpeedWeight = default(1.20);
        double contextYawWeight = default(0.035);
}
