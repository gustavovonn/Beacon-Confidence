#include "sdDynamicState.h"
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
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 

#define MAXLINE 1024
#define FLOAT_INT_PRECISION 100000000. //# float int precision for longitude and latitude in LLA (rad) --> 1e8 because 1e9 would overflow for values close to -180 or 180.
#define FLOAT_INT_PRECISION_M 1000. //# float int precision for altitude in LLA (meters)

#define BOOST_GEOMETRY_DISABLE_DEPRECATED_03_WARNING

namespace artery{
using namespace omnetpp;
using namespace vanetza;
}
# define M_PI           3.14159265358979323846  /* pi */

auto microdegree7 = vanetza::units::degree * boost::units::si::micro;

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}

Define_Module(SDDynamicState)

void SDDynamicState::initialize(){

    artery::ItsG5BaseService::initialize();
    _mVehicleController = &getFacilities().get_const<traci::VehicleController>();
    _mVehicleDataProvider = &getFacilities().get_const<artery::VehicleDataProvider>();

    const auto cfgex = getSimulation()->getEnvir()->getConfigEx();
    const auto cfg_run_number = cfgex->getVariable("runnumber");
    const auto timelimit = getSimulation()->getEnvir()->getConfig()->getConfigValue("sim-time-limit");
    runnumber= std::string(cfg_run_number);
    std::string folder("results/results_"+runnumber);

    smartdataEnabled = par("smartdataEnabled");

    if(smartdataEnabled){
        sdip = "224.1.1.1";
        sdport = 5000+1+16;
        sdport_emission = 5000+1+42;
        sdport_fuel = 5000+1+43;
        // _MULTICAST_TTL = 2

        // Creating socket file descriptor 
        if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
            perror("socket creation failed"); 
            exit(EXIT_FAILURE); 
        }

        if ( (sockfd_emission = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
            perror("socket emission creation failed"); 
            exit(EXIT_FAILURE); 
        }

        if ( (sockfd_fuel = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
            perror("socket fuel emission creation failed"); 
            exit(EXIT_FAILURE); 
        }

        // Filling server information 
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = inet_addr(sdip.c_str());
        servaddr.sin_port = htons(sdport);

        servaddr_emission.sin_family = AF_INET;
        servaddr_emission.sin_addr.s_addr = inet_addr(sdip.c_str());
        servaddr_emission.sin_port = htons(sdport_emission); 

        servaddr_fuel.sin_family = AF_INET;
        servaddr_fuel.sin_addr.s_addr = inet_addr(sdip.c_str());
        servaddr_fuel.sin_port = htons(sdport_fuel); 
        
    }

    const int dir_err = mkdir(folder.c_str() , S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

}

void SDDynamicState::sendsd(unsigned char * data, int size, struct sockaddr_in servaddr){
    sendto(sockfd, (const char *)data, size, 
            MSG_CONFIRM, (const struct sockaddr *) &servaddr,  
                sizeof(servaddr)); 
    // std::cout<<"Hello message sent."<<std::endl; 
}

std::string SDDynamicState::receivesd(){
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

void SDDynamicState::copyto(unsigned char dest[], float data, int pos){
    unsigned char temp[4];

    memcpy(temp, &(data), 4);

    dest[pos]=temp[0];
    dest[pos+1]=temp[1];
    dest[pos+2]=temp[2];
    dest[pos+3]=temp[3];
}

void SDDynamicState::copyto(unsigned char dest[], int data, int pos){
    unsigned char temp[4];

    memcpy(temp, &(data), 4);

    dest[pos]=temp[0];
    dest[pos+1]=temp[1];
    dest[pos+2]=temp[2];
    dest[pos+3]=temp[3];
}

void SDDynamicState::copyto(unsigned char dest[], std::vector<unsigned char> data, int pos){
    
    for(int i=0; i< data.size(); i++){
        dest[pos+i]=data[i];
    }
}

std::vector<unsigned char> SDDynamicState::intToBytes(int paramInt, int size)
{
     std::vector<unsigned char> arrayOfByte(size);
     for (int i = 0; i < size; i++)
         arrayOfByte[size-1 - i] = (paramInt >> (i * 8));
     return arrayOfByte;
}

std::vector<unsigned char> SDDynamicState::intToBytes(unsigned long long paramInt, int size)
{
     std::vector<unsigned char> arrayOfByte(size);
     for (int i = 0; i < size; i++)
         arrayOfByte[size-1 - i] = (paramInt >> (i * 8));
     return arrayOfByte;
}

void SDDynamicState::trigger(){

    Enter_Method("SDDynamicState trigger");

    Motion_Vector_Global s;

    s._id = _mVehicleDataProvider->station_id();
    s._location._lat._value = round(_mVehicleDataProvider->latitude(), microdegree7) * Longitude_oneMicrodegreeEast * (M_PI/180.0); // Latitude_oneMicrodegreeNorth = 10 --> positive
    s._location._lat._uncertainty = 0; // TODO: from elipse to uncertainty
    s._location._lon._value = round(_mVehicleDataProvider->longitude(), microdegree7) * Latitude_oneMicrodegreeNorth * (M_PI/180.0);
    s._location._lon._uncertainty = 0;
    s._location._alt._value = 0; // AltitudeValue_unavailable
    s._location._alt._uncertainty = 100;
    s._speed._value = _mVehicleDataProvider->speed().value();
    s._speed._uncertainty = 0;
    s._heading._value = _mVehicleDataProvider->heading().value();   // heading is already in radians
    // std::cout<<"mv: "<<s._heading._value<<" "<<_mVehicleDataProvider->heading().value()<<"\n";
    s._heading._uncertainty = 0;
    s._yaw_rate._value = _mVehicleDataProvider->yaw_rate().value(); // rad/s
    s._yaw_rate._uncertainty = 0;
    s._acceleration._value = _mVehicleDataProvider->acceleration().value(); // / vanetza::units::si::meter_per_second_squared;
    s._acceleration._uncertainty = 0;
    s._dimension._length._value = _mVehicleController->getLength().value();
    s._dimension._length._uncertainty = 0;
    s._dimension._width._value = _mVehicleController->getWidth().value();
    s._dimension._width._uncertainty = 0;
    s._dimension._height._value = 0;
    s._dimension._height._value = 100;
    // s._steering._value = 90;
    // s._steering._uncertainty = 100;
    // s._orientation._value = s._heading._value;
    // s._orientation._uncertainty = s._heading._uncertainty;
    // s._parking = false;
    s._class = 5; // StationType_passengerCar
    s._uncertainty = 0;
    s._timestamp = _mVehicleDataProvider->updated().inUnit(omnetpp::SIMTIME_US);

    if(smartdataEnabled){

        unsigned char data[54+sizeof(unsigned long long)];

        data[0]='\x01';

        // location=[int(lon*FLOAT_INT_PRECISION), int(lat*FLOAT_INT_PRECISION), int(alt*FLOAT_INT_PRECISION_M)],
        
        int lon = (int) (s._location._lon._value*100);
        memcpy(&data[1], reinterpret_cast<unsigned char *> (&lon), sizeof(int) );
        int lat = (int) (s._location._lat._value*100);
        memcpy(&data[5], reinterpret_cast<unsigned char *> (&lat), sizeof(int) );
        int alt = (int) (s._location._alt._value*FLOAT_INT_PRECISION_M);
        memcpy(&data[9], reinterpret_cast<unsigned char *> (&alt), sizeof(int) );

        //speed
        memcpy(&data[13], reinterpret_cast<unsigned char *> (&s._speed._value), sizeof(float) );
        //heading
        memcpy(&data[17], reinterpret_cast<unsigned char *> (&s._heading._value), sizeof(float) );
        //yaw rate
        memcpy(&data[21], reinterpret_cast<unsigned char *> (&s._yaw_rate._value), sizeof(float) );

        // acceleration
        memcpy(&data[25], reinterpret_cast<unsigned char *> (&s._acceleration._value), sizeof(float) );
        //ID
        // std::cout<<"ID: "<<s._id<<"\n";
        unsigned long long tempid= s._id;
        memcpy(&data[29], reinterpret_cast<unsigned char *> (&tempid), sizeof(unsigned long long) );
    
        int temp=0;
        //uncertainty
        memcpy(&data[37], reinterpret_cast<unsigned char *> (&temp), sizeof(float) );

        temp=12;
        //class: 'Reference'  : 12
        memcpy(&data[41], reinterpret_cast<unsigned char *> (&temp), sizeof(float) );

        //timestamp
        memcpy(&data[45], reinterpret_cast<unsigned char *> (&s._timestamp), sizeof(unsigned long long) );

        data[53]='\x00';
        memcpy(&data[54], reinterpret_cast<unsigned char *> (&tempid), sizeof(unsigned long long) );
        sendsd(data, 54+sizeof(unsigned long long), servaddr);

        // std::cout << _mVehicleController->getTraCI()->vehicle.getCO2Emission(_mVehicleController->getVehicleId()) << std::endl;
        // std::cout << _mVehicleController->getTraCI()->vehicle.getFuelConsumption(_mVehicleController->getVehicleId()) << std::endl;

        double co2_emission = _mVehicleController->getTraCI()->vehicle.getCO2Emission(_mVehicleController->getVehicleId());

        unsigned char emission_data[16];
        memcpy(&emission_data[0], reinterpret_cast<unsigned char *> (&co2_emission), sizeof(double) );
        memcpy(&emission_data[8], reinterpret_cast<unsigned char *> (&tempid), sizeof(unsigned long long) );
        sendsd(emission_data, 16, servaddr_emission);

        double fuel_consumption = _mVehicleController->getTraCI()->vehicle.getFuelConsumption(_mVehicleController->getVehicleId());

        unsigned char fuel_data[16];
        memcpy(&fuel_data[0], reinterpret_cast<unsigned char *> (&fuel_consumption), sizeof(double) );
        memcpy(&fuel_data[8], reinterpret_cast<unsigned char *> (&tempid), sizeof(unsigned long long) );
        sendsd(fuel_data, 16, servaddr_fuel);
        
    }

    if(fileRecord){
        std::ofstream myfile;
        myfile.open("results/results_"+runnumber+"/monitor_car-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
        myfile << s.to_string();

        // myfile << "CO2 Emission: " << co2_emission << std::endl;
        // myfile << "Fuel Consumption: " << fuel_consumption << std::endl;
        myfile.close();

        myfile.open("results/results_"+runnumber+"/localperceptionGT-vehicle_0.out",  std::ios_base::app);
        myfile << s.to_string();
        myfile.close();
    }
}
