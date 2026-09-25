/*
* Artery V2X Simulation Framework
* Copyright 2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#ifndef CollectivePerceptionService_H_
#define CollectivePerceptionService_H_

#include "artery/traci/VehicleController.h"
#include "artery/envmod/service/CollectivePerceptionMockMessage.h"
#include <vanetza/asn1/cpm.hpp>
#include "artery/application/ItsG5BaseService.h"
#include "artery/networking/PositionProvider.h"
#include <unordered_set>
#include <vector>
#include "artery/utility/Channel.h"
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include "artery/application/VehicleDataProvider.h"
#include "artery/utility/IdentityRegistry.h"
// #include "artery/utility/Geometry.h"



class CollectivePerceptionService : public artery::ItsG5BaseService
{
    public:
        virtual ~CollectivePerceptionService();
        CollectivePerceptionService();
        const artery::VehicleDataProvider* mVehicleDataProvider = nullptr;

    protected:
        int getObjectID(int badid);
        int numInitStages() const override;
        void initialize(int stage) override;
        void trigger() override;
        void handleMessage(omnetpp::cMessage*) override;
        void generatePacket();
        void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
        void addPerceptionObjects(vanetza::asn1::Cpm& message);
        omnetpp::SimTime genCpmDcc();
        bool checkHeadingDelta() const;
		bool checkPositionDelta() const;
		bool checkSpeedDelta() const;

        vanetza::asn1::Cpm createCollectivePerceptionMessage(const artery::VehicleDataProvider& vdp, uint16_t genDeltaTime);

    private:
        const artery::GeoPosition* mGeoPosition = nullptr;
        enum StationType mStationType;
        omnetpp::SimTime mGenCpmMin;
		omnetpp::SimTime mGenCpmMax;
		omnetpp::SimTime mGenCpm;
        bool mDccRestriction;
        omnetpp::SimTime mLastCpmTimestamp;
        const artery::NetworkInterfaceTable* mNetworkInterfaceTable = nullptr;
        artery::ChannelNumber mPrimaryChannel = artery::channel::CCH;
        const traci::VehicleController* _mVehicleController = nullptr;
        const artery::Timer* mTimer = nullptr;
        int mHostId = 0;
        const artery::PositionProvider* mPositionProvider = nullptr;
        const artery::LocalEnvironmentModel* mEnvironmentModel = nullptr;
        const artery::LocalEnvironmentModel* localEnvmod;
        omnetpp::cMessage* mTrigger = nullptr;
        bool mGenerateAfterCam;
        omnetpp::SimTime mCpmOffset;
        omnetpp::SimTime mCpmInterval;
        omnetpp::SimTime mFovInterval = omnetpp::SimTime::ZERO;
        omnetpp::SimTime mFovLast = omnetpp::SimTime::ZERO;
        std::vector<artery::CollectivePerceptionMockMessage::FovContainer> mFovContainers;
        std::unordered_set<const artery::Sensor*> mSensors;
        unsigned mDccProfile = 0;
        unsigned mLengthHeader = 0;
        unsigned mLengthFovContainer = 0;
        unsigned mLengthObjectContainer = 0;
        bool mFixedRate;
        double mSensorRange;
        artery::IdentityRegistry* mIdentityRegistry;
        std::string runnumber;

        artery::Position mLastCpmPosition;
		vanetza::units::Velocity mLastCpmSpeed;
		vanetza::units::Angle mLastCpmHeading;
        vanetza::units::Angle mHeadingDelta;
		vanetza::units::Length mPositionDelta;
		vanetza::units::Velocity mSpeedDelta;

        unsigned mGenCpmLowDynamicsCounter;
		unsigned mGenCpmLowDynamicsLimit;
};


#endif /* COLLECTIVEPERCEPTIONSERVICE_H */
