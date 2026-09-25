#ifndef MonitorCollectivePerception_H_
#define MonitorCollectivePerception_H_

#include "motion_vector.h"
#include "CollectivePerceptionService.h"
#include "udp_socket.h"
#include <omnetpp/simtime.h>
#include "artery/application/VehicleDataProvider.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/envmod/EnvironmentModelObject.h"
#include "LocalPerceptionNode.h"
#include "artery/envmod/GlobalEnvironmentModel.h"
#include "CpObject.h"
#include "artery/utility/IdentityRegistry.h"

// class artery::Timer;
// class artery::VehicleDataProvider;

class MonitorCollectivePerception : public CollectivePerceptionService
{
    public:
        static const bool BINARY_SAVE = false;
        static const unsigned int SD_DEVICE = 26;
        const artery::Timer* mTimer = nullptr;
        const artery::VehicleDataProvider* mVehicleDataProvider = nullptr;

    public:
		void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
        void trigger() override;
        MonitorCollectivePerception() : CollectivePerceptionService() {}


    protected:
        void initialize(int stage) override;
        int getObjectID(int badid);

    private:
    
        const traci::VehicleController* _mVehicleController = nullptr;
        const artery::VehicleDataProvider* _mVehicleDataProvider = nullptr;
        LocalPerceptionNode* _lpn;
        UDP_Socket* _socket = nullptr;
        artery::GlobalEnvironmentModel* globalEnvmod = nullptr;
        artery::IdentityRegistry* mIdentityRegistry;
        std::string runnumber;

        void sendsd(unsigned char * data, int size);
        std::string receivesd();
        int sdport;
        std::string sdip;
        struct sockaddr_in servaddr;
        int sockfd;
        bool smartdataEnabled;
};

#endif /* MonitorCollectivePerception_H_ */