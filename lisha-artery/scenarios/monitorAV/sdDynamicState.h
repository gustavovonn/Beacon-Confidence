#ifndef SDDynamicState_H_
#define SDDynamicState_H_

#include "artery/application/ItsG5Service.h"
#include "artery/utility/Channel.h"
#include "artery/utility/Geometry.h"
#include <vanetza/asn1/cam.hpp>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>
#include "udp_socket.h"
#include "motion_vector.h"

#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>

// forward declaration

namespace traci { class VehicleController; }
namespace artery
{ 
    class VehicleDataProvider;
}
    

class SDDynamicState : public artery::ItsG5Service
{

    public:
        void trigger() override;

    protected:
        void initialize() override;
    
    private:
        void copyto(unsigned char dest[], float data, int pos);
        void copyto(unsigned char dest[], int data, int pos);
        void copyto(unsigned char dest[], std::vector<unsigned char> data, int pos);
        std::vector<unsigned char> intToBytes(int paramInt, int size);
        std::vector<unsigned char> intToBytes(unsigned long long paramInt, int size);
        void sendsd(unsigned char * data, int size, struct sockaddr_in);
        std::string receivesd();
        const traci::VehicleController* _mVehicleController = nullptr;
        const artery::VehicleDataProvider* _mVehicleDataProvider = nullptr;
        UDP_Socket* _socket = nullptr;
        std::string runnumber;
        bool fileRecord;
        bool smartdataEnabled;
        std::string iotURL;
        int sdUploadCount;
        std::vector<Motion_Vector_Global> mvlist;
        std::string sdchannel;
        int sdchannelid;
        std::string sdcontrol;
        int sdcontrolid;
        std::string sdip;
        int sdport;
        struct sockaddr_in servaddr;
        int sockfd;

        int sdport_emission;
        struct sockaddr_in servaddr_emission;
        int sockfd_emission;

        int sockfd_fuel;
        struct sockaddr_in servaddr_fuel;
        int sdport_fuel;
};


#endif /* SDDynamicState_H_ */
