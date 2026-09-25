#ifndef MONITORCA_H_
#define MONITORCA_H_

#include "motion_vector.h"
#include "artery/application/CaService.h"
#include "udp_socket.h"
#include <omnetpp/simtime.h>
#include "artery/application/VehicleDataProvider.h"
#include "artery/traci/VehicleController.h"

#include "LocalPerceptionNode.h"
#include "artery/envmod/GlobalEnvironmentModel.h"

// class artery::Timer;
// class artery::VehicleDataProvider;

class MonitorCa : public artery::CaService
{
    public:
        static const bool BINARY_SAVE = false;
        static const unsigned int SD_DEVICE = 26;
        const artery::Timer* mTimer = nullptr;
        const artery::VehicleDataProvider* mVehicleDataProvider = nullptr;
        artery::GlobalEnvironmentModel* globalEnvmod = nullptr;

    public:
		void trigger() override;
		void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
        MonitorCa() : CaService() {}

    protected:
        void initialize() override;

    private:
        void sendsd(unsigned char * data, int size);
        std::string receivesd();
        const traci::VehicleController* _mVehicleController = nullptr;
        LocalPerceptionNode* _lpn;
        UDP_Socket* _socket = nullptr;
        std::string runnumber;
        int sdport;
        std::string sdip;
        struct sockaddr_in servaddr;
        int sockfd;
        bool smartdataEnabled;
};

#endif /* MONITORCA_H_ */