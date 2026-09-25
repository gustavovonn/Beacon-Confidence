#include "SD_template.h"
#include "artery/traci/VehicleController.h"
#include "artery/application/VehicleDataProvider.h"
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>
#include <iostream>
#include <fstream>
#include <math.h> 
#include <boost/math/constants/constants.hpp>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/utility/simtime_cast.h"
#include "veins/base/utils/Coord.h"
#include <omnetpp/cexception.h>
#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <chrono>
#include <omnetpp/cconfiguration.h>
#include <sys/select.h>
#include <errno.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>


#define BOOST_GEOMETRY_DISABLE_DEPRECATED_03_WARNING

#define MAXLINE 1024
#define FLOAT_INT_PRECISION 100000000. //# float int precision for longitude and latitude in LLA (rad) --> 1e8 because 1e9 would overflow for values close to -180 or 180.
#define FLOAT_INT_PRECISION_M 1000. //# float int precision for altitude in LLA (meters)

namespace artery{
using namespace omnetpp;
using namespace vanetza;
}
# define M_PI           3.14159265358979323846  /* pi */

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}

Define_Module(SD_template)

void SD_template::initialize(){

    artery::ItsG5BaseService::initialize();


    // vehicle controller and data provider are two instances in artery that can provide some vehicular data
    _mVehicleController = &getFacilities().get_const<traci::VehicleController>();
    _mVehicleDataProvider = &getFacilities().get_const<artery::VehicleDataProvider>();

    // .ned parameters can be read with the par(string) function
    fileRecord = par("fileRecord");
    smartdataEnabled = par("smartdataEnabled");
    iotURL = std::string(par("iotURL").stringValue());
    sdUploadCount = par("sdUploadCount");

    // if smartdata is enabled we configure the socket
    if(smartdataEnabled){
        sdip = "224.1.1.1";
        sdport = 5000+1+par("port").intValue();

        // Creating socket file descriptor 
        if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
            perror("socket creation failed"); 
            exit(EXIT_FAILURE); 
        }

        // Filling server information 
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = inet_addr(sdip.c_str());
        servaddr.sin_port = htons(sdport); 
        
    }

}

void SD_template::sendsd(unsigned char * data, int size){
    sendto(sockfd, (const char *)data, size, 
            MSG_CONFIRM, (const struct sockaddr *) &servaddr,  
                sizeof(servaddr)); 
    // std::cout<<"Hello message sent."<<std::endl; 
}

std::string SD_template::receivesd(){
    char buffer[MAXLINE];
    int n;
    socklen_t len;

    n = recvfrom(sockfd, (char *)buffer, MAXLINE,  
                MSG_WAITALL, (struct sockaddr *) &servaddr, 
                &len); 
    buffer[n] = '\0'; 
    // std::cout<<"Server :"<<buffer<<std::endl; 

    std::string retstr(buffer);

    return retstr;
}

// The trigger function executes periodically
void SD_template::trigger(){

    Enter_Method("SD_template trigger");

    if(smartdataEnabled){
        
        
    }

}

// the indicate function handles received messages
void SD_template::indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>){

    if(smartdataEnabled){
        
        
    }

}
