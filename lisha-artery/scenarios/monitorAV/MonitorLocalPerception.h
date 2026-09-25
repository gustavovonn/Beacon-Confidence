/*
* Artery V2X Simulation Framework
* Copyright 2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#ifndef MONITORSERVICE_H
#define MONITORSERVICE_H

#include "artery/application/ItsG5Service.h"
#include "artery/networking/PositionProvider.h"
#include "motion_vector.h"
#include <unordered_set>
#include <vector>
#include <iostream>
#include <fstream>
#include "LocalPerceptionNode.h"

namespace artery { class LocalEnvironmentModel; }

class MonitorLocalPerception : public artery::ItsG5Service
{
    public:
        virtual ~MonitorLocalPerception();

    protected:
        void initialize(int stage) override;
        void trigger() override;

    private:
        double mSensorRange;
        LocalPerceptionNode* _lpn;
        const traci::VehicleController* _mVehicleController = nullptr;
        const artery::VehicleDataProvider* _mVehicleDataProvider = nullptr;
        int mHostId = 0;
        const artery::PositionProvider* mPositionProvider = nullptr;
        const artery::LocalEnvironmentModel* localEnvmod;
        omnetpp::cMessage* mTrigger = nullptr;
        std::unordered_set<const artery::Sensor*> mSensors;
        std::string runnumber;
};

// namespace artery

#endif /* MONITORSERVICE_H */
