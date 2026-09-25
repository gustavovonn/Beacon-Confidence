/*
* Artery V2X Simulation Framework
* Copyright 2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#include "artery/envmod/sensor/FovSensor.h"
#include "MonitorLocalPerception.h"
#include "artery/utility/InitStages.h"
#include "artery/utility/PointerCheck.h"
#include <omnetpp/checkandcast.h>
#include <omnetpp/cmessage.h>
#include <omnetpp/cpacket.h>
#include <memory>

#include "artery/envmod/service/CollectivePerceptionMockMessage.h"
#include "artery/envmod/service/CollectivePerceptionMockService.h"
#include "artery/traci/VehicleController.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/envmod/EnvironmentModelObject.h"
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/cconfiguration.h>
#include <sys/stat.h>

#define BOOST_GEOMETRY_DISABLE_DEPRECATED_03_WARNING

auto microdegree3 = vanetza::units::degree * boost::units::si::micro;

# define M_PI           3.14159265358979323846  /* pi */

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}


Define_Module(MonitorLocalPerception)

MonitorLocalPerception::~MonitorLocalPerception()
{
    cancelAndDelete(mTrigger);
}

void MonitorLocalPerception::initialize(int stage)
{
    artery::ItsG5BaseService::initialize();
    _mVehicleController = &getFacilities().get_const<traci::VehicleController>();
    _mVehicleDataProvider = &getFacilities().get_const<artery::VehicleDataProvider>();
    localEnvmod = &getFacilities().get_const<artery::LocalEnvironmentModel>();
    _lpn = &getFacilities().get_mutable<LocalPerceptionNode>();
    mSensorRange = par("sensorRange");

    // getFacilities().registerMutable

    const auto cfg = getSimulation()->getEnvir()->getConfigEx();
    const auto cfg_run_number = cfg->getVariable("runnumber");
    runnumber= std::string(cfg_run_number);

    std::string folder("results/results_"+runnumber);

    const int dir_err = mkdir(folder.c_str() , S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    std::ofstream myfile;
    myfile.open("results/results_"+runnumber+"/perception-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::trunc);
    myfile.close();
}


void MonitorLocalPerception::trigger()
{
    Enter_Method("MonitorLocalPerception trigger");
    
    std::ofstream myfile;
    myfile.open("results/results_"+runnumber+"/perception-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
    

    using TrackedObject = artery::LocalEnvironmentModel::TrackedObject;

    auto& m_sim = _mVehicleController->getTraCI()->simulation;
    auto vehpos = m_sim.convertGeo(_mVehicleDataProvider->latitude().value(), _mVehicleDataProvider->longitude().value(), true);

    int i=0;
    
    for (const auto& object : localEnvmod->allObjects()) {
        i++;
        const artery::LocalEnvironmentModel::Tracking& tracking = object.second;
        if (tracking.expired()) {
            // skip objects with lost tracking
            continue;
        }

        for (const auto& sensor : tracking.sensors()) {

            std::shared_ptr<artery::EnvironmentModelObject> obj=object.first.lock();

            if(!obj){
                continue;
            }
            else{

                const artery::VehicleDataProvider* objDataProvider=&(obj->getVehicleData());

                auto objpos = m_sim.convertGeo(objDataProvider->latitude().value(), objDataProvider->longitude().value(), true);

                if( sqrt(pow(objpos.x-vehpos.x, 2) + pow(objpos.y-vehpos.y, 2) * 1.0) > mSensorRange)
                    continue;

                Motion_Vector_Global s;

                s._id = objDataProvider->station_id();;
                s._location._lat._value = round(objDataProvider->latitude(), microdegree3) * Longitude_oneMicrodegreeEast * (M_PI/180.0); // Latitude_oneMicrodegreeNorth = 10 --> positive
                // s._location._lat._value = objDataProvider->latitude().value(); // Latitude_oneMicrodegreeNorth = 10 --> positive
                s._location._lat._uncertainty = 0; // TODO: from elipse to uncertainty
                s._location._lon._value = round(objDataProvider->longitude(), microdegree3) * Latitude_oneMicrodegreeNorth * (M_PI/180.0);
                // s._location._lon._value = objDataProvider->longitude().value();
                s._location._lon._uncertainty = 0;
                s._location._alt._value = 0; // AltitudeValue_unavailable
                s._location._alt._uncertainty = 100;
                s._speed._value = objDataProvider->speed().value();
                s._speed._uncertainty = 0;
                s._heading._value = objDataProvider->heading().value();   // heading is already in radians
                s._heading._uncertainty = 0;
                s._yaw_rate._value = objDataProvider->yaw_rate().value(); // rad/s
                s._yaw_rate._uncertainty = 0;
                s._acceleration._value = objDataProvider->acceleration().value(); // / vanetza::units::si::meter_per_second_squared;
                s._acceleration._uncertainty = 0;
                s._dimension._length._value = _mVehicleController->getLength().value();
                s._dimension._length._uncertainty = 0;
                s._dimension._width._value = _mVehicleController->getWidth().value();
                s._dimension._width._uncertainty = 0;
                s._dimension._height._value = 0;
                s._dimension._height._value = 100;
                s._class = 5; // StationType_passengerCar
                s._uncertainty = 0;
                s._timestamp = objDataProvider->updated().inUnit(omnetpp::SIMTIME_US);

                myfile << s.to_string();

                object_pair newdata;
                newdata.mv=s;
                // newdata.objDataProvider=objDataProvider;

                _lpn->addDataLocal(newdata);
            }
        }
        myfile.close();
    }
}

 // namespace artery
