#include "MonitorMV.h"
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

Define_Module(MonitorMV)

void MonitorMV::initialize(){

    artery::ItsG5BaseService::initialize();
    _mVehicleController = &getFacilities().get_const<traci::VehicleController>();
    _mVehicleDataProvider = &getFacilities().get_const<artery::VehicleDataProvider>();

    const auto cfgex = getSimulation()->getEnvir()->getConfigEx();
    const auto cfg_run_number = cfgex->getVariable("runnumber");
    const auto timelimit = getSimulation()->getEnvir()->getConfig()->getConfigValue("sim-time-limit");
    runnumber= std::string(cfg_run_number);
    std::string folder("results/results_"+runnumber);

    fileRecord = par("fileRecord");
    smartdataEnabled = par("smartdataEnabled");
    iotURL = std::string(par("iotURL").stringValue());
    sdUploadCount = par("sdUploadCount");

    if(smartdataEnabled){
        std::vector<std::string> seriessd = initSD(iotURL, _mVehicleDataProvider->updated().inUnit(omnetpp::SIMTIME_US), _mVehicleDataProvider->updated().inUnit(omnetpp::SIMTIME_US), std::to_string(_mVehicleDataProvider->station_id()));
        sdchannel = "/tmp/sdchannel";
        sdcontrol = "/tmp/sdcontrol";
        mkfifo(sdchannel.c_str(), 0666);
        mkfifo(sdcontrol.c_str(), 0666);

        std::string start = "Connect "+std::string(timelimit)+"\n";

        sdchannelid = open(sdchannel.c_str(), O_WRONLY ); //open(myfifo, O_WRONLY | O_NONBLOCK);
        if (sdchannelid == -1) {
            perror("open");
            return;
        }
        write(sdchannelid, start.c_str(), start.size());
        close(sdchannelid);

        sdcontrolid = open(sdcontrol.c_str(), O_RDONLY );
        char buffer[10];
        int test = read(sdcontrolid, &buffer[0], 10);
        while(test!=0) test = read(sdcontrolid, &buffer[0], 10);
        close(sdcontrolid);
        memset(buffer, 0, 10);

        for(int i=0; i<seriessd.size(); i++){
            sdchannelid = open(sdchannel.c_str(), O_WRONLY ); //open(myfifo, O_WRONLY | O_NONBLOCK);
            if (sdchannelid == -1) {
                perror("open");
                return;
            }
            write(sdchannelid, seriessd[i].c_str(), seriessd[i].size());
            close(sdchannelid);

            sdcontrolid = open(sdcontrol.c_str(), O_RDONLY );
            char buffer[10];
            int test = read(sdcontrolid, &buffer[0], 10);
            while(test!=0) test = read(sdcontrolid, &buffer[0], 10);
            close(sdcontrolid);
            memset(buffer, 0, 10);
        }
    }

    const int dir_err = mkdir(folder.c_str() , S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    if(fileRecord){
        std::ofstream myfile;
        myfile.open("results/results_"+runnumber+"/monitor_car-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::trunc);
        myfile.close();
        myfile.open("results/results_"+runnumber+"/UKFtest-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::trunc);
        myfile.close();
    }
}

void MonitorMV::trigger(){

    Enter_Method("MonitorMV trigger");

    Motion_Vector_Global s;

    s._id = _mVehicleDataProvider->station_id();
    s._location._lat._value = round(_mVehicleDataProvider->latitude(), microdegree) * Longitude_oneMicrodegreeEast * (M_PI/180.0); // Latitude_oneMicrodegreeNorth = 10 --> positive
    s._location._lat._uncertainty = 0; // TODO: from elipse to uncertainty
    s._location._lon._value = round(_mVehicleDataProvider->longitude(), microdegree) * Latitude_oneMicrodegreeNorth * (M_PI/180.0);
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


    // auto& m_sim = _mVehicleController->getTraCI()->simulation;
    // std::cout<<"\nstarting\n";
    // std::cout<<"car lat:"<<_mVehicleDataProvider->latitude().value()<<" lon:"<< _mVehicleDataProvider->longitude().value()<<"\n";
    // auto carPos = m_sim.convertGeo(_mVehicleDataProvider->latitude().value(), _mVehicleDataProvider->longitude().value(), true);

    // std::cout<<"car pos x:"<<carPos.x<<" y:"<< carPos.y<<"\n";
    // carPos.x+=3;
    // std::cout<<"newcar pos x:"<<carPos.x<<" y:"<< carPos.y<<"\n";

    // auto newgeo = m_sim.convertGeo(carPos.x, carPos.y, false);
    // std::cout<<"newcar lat:"<<newgeo.x<<" lon:"<< newgeo.y<<"\n";

    // auto newpos = m_sim.convertGeo(newgeo.x, newgeo.y, true);
    // std::cout<<"newcar conv x:"<<newpos.x<<" y:"<< newpos.y<<"\n";
    // std::ofstream myfile;
    // myfile.open("results/results_"+runnumber+"/UKFtest-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
    // auto& m_sim = _mVehicleController->getTraCI()->simulation;
    // auto carPos = m_sim.convertGeo(_mVehicleDataProvider->longitude().value(), _mVehicleDataProvider->latitude().value(), true);

    // myfile<<"L\t"<<carPos.x<<"\t"<<carPos.y<<"\t"<<_mVehicleDataProvider->speed().value()<<"\t"<<((M_PI/2)-_mVehicleDataProvider->heading().value())<<"\t"<<_mVehicleDataProvider->updated().inUnit(omnetpp::SIMTIME_US);
    // myfile<<"\t"<<carPos.x<<"\t"<<carPos.y<<"\t"<<_mVehicleDataProvider->speed().value()* cos(((M_PI/2)-_mVehicleDataProvider->heading().value()))<<"\t"<<_mVehicleDataProvider->speed().value()* sin(((M_PI/2)-_mVehicleDataProvider->heading().value()))<<"\t"<<"\t"<<((M_PI/2)-_mVehicleDataProvider->heading().value())<<"\t"<<_mVehicleDataProvider->yaw_rate().value()<<"\n";
    // myfile.close();

    // mvlist.push_back(s);

    if(smartdataEnabled){
        std::vector<std::string> seriessd = uploadSD(iotURL, s);
        // mvlist.clear();
        
        for(int i=0; i<seriessd.size(); i++){
            sdchannelid = open(sdchannel.c_str(), O_WRONLY ); //open(myfifo, O_WRONLY | O_NONBLOCK);
            if (sdchannelid == -1) {
                perror("open");
                return;
            }
            write(sdchannelid, seriessd[i].c_str(), seriessd[i].size());
            close(sdchannelid);

            sdcontrolid = open(sdcontrol.c_str(), O_RDONLY );
            char buffer[10];
            int test = read(sdcontrolid, &buffer[0], 10);
            while(test!=0) test = read(sdcontrolid, &buffer[0], 10);
            close(sdcontrolid);
            memset(buffer, 0, 10);
        }
        
    }

    if(fileRecord){
        std::ofstream myfile;
        myfile.open("results/results_"+runnumber+"/monitor_car-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
        myfile << s.to_string();
        myfile.close();

        // myfile.open("results/results_"+runnumber+"/localperceptionGT-vehicle_0.out",  std::ios_base::app);
        // myfile << s.to_string();
        // myfile.close();
    }

}
