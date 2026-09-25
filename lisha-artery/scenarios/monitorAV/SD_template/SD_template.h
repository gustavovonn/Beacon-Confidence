#ifndef SD_template_H_
#define SD_template_H_

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

auto microdegree = vanetza::units::degree * boost::units::si::micro;
    

class SD_template : public artery::ItsG5Service
{

    public:
        void trigger() override;
        void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;

    protected:
        void initialize() override;
    
    private:
        const traci::VehicleController* _mVehicleController = nullptr;
        const artery::VehicleDataProvider* _mVehicleDataProvider = nullptr;
        UDP_Socket* _socket = nullptr;
        std::string runnumber;
        bool fileRecord;
        std::string iotURL;
        int sdUploadCount;
        std::vector<Motion_Vector_Global> mvlist;

        void sendsd(unsigned char * data, int size);
        std::string receivesd();
        int sdport;
        std::string sdip;
        struct sockaddr_in servaddr;
        int sockfd;
        bool smartdataEnabled;
};


#endif /* SD_template_H_ */
