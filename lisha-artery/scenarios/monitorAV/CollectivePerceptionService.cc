/*
* Artery V2X Simulation Framework
* Copyright 2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#include "artery/envmod/sensor/FovSensor.h"
#include "artery/envmod/service/CollectivePerceptionMockMessage.h"
#include "CollectivePerceptionService.h"
#include "artery/utility/InitStages.h"
#include "artery/utility/PointerCheck.h"
#include <omnetpp/checkandcast.h>
#include <omnetpp/cmessage.h>
#include <omnetpp/cpacket.h>

#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include "CpObject.h"
#include "artery/application/Asn1PacketVisitor.h"
#include <boost/units/systems/si/prefixes.hpp>
#include <fstream>
#include <boost/units/cmath.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <inet/common/ModuleAccess.h>

auto microdegree6 = vanetza::units::degree * boost::units::si::micro;

# define M_PI           3.14159265358979323846  /* pi */

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}

using namespace omnetpp;

Define_Module(CollectivePerceptionService)

namespace
{

// omnetpp::simsignal_t cpmSentSignal = omnetpp::cComponent::registerSignal("CpmSent");
// omnetpp::simsignal_t cpmReceivedSignal = omnetpp::cComponent::registerSignal("CpmReceived");

static const simsignal_t cpmReceivedSignal = cComponent::registerSignal("CpmReceived");
static const simsignal_t cpmSentSignal = cComponent::registerSignal("CpmSent");

} // namespace

CollectivePerceptionService::CollectivePerceptionService() :
    mGenCpmMin { 100, SIMTIME_MS },
    mGenCpmMax { 1000, SIMTIME_MS },
    mGenCpm(mGenCpmMax),
    mGenCpmLowDynamicsCounter(0),
    mGenCpmLowDynamicsLimit(3)
{
}


CollectivePerceptionService::~CollectivePerceptionService()
{
    cancelAndDelete(mTrigger);
}

int CollectivePerceptionService::numInitStages() const
{
    return artery::InitStages::Total;
}

void CollectivePerceptionService::initialize(int stage)
{
    if (stage == artery::InitStages::Prepare) {
        ItsG5BaseService::initialize();
        mPositionProvider = &getFacilities().get_const<artery::PositionProvider>();
        mEnvironmentModel = &getFacilities().get_const<artery::LocalEnvironmentModel>();

        mDccProfile = par("dccProfile");
        mLengthHeader = par("lengthHeader");
        mLengthFovContainer = par("lengthFovContainer");
        mLengthObjectContainer = par("lengthObjectContainer");

        mDccRestriction = par("withDccRestriction");

        mGenCpmMin = par("minInterval");
        mGenCpmMax = par("maxInterval");
        mGenCpm = mGenCpmMax;

        mFixedRate = par("fixedRate");

        mSensorRange = par("sensorRange");

        // vehicle dynamics thresholds
        mHeadingDelta = vanetza::units::Angle { par("headingDelta").doubleValue() * vanetza::units::degree };
        mPositionDelta = par("positionDelta").doubleValue() * vanetza::units::si::meter;
        mSpeedDelta = par("speedDelta").doubleValue() * vanetza::units::si::meter_per_second;

        mGenerateAfterCam = par("generateAfterCam");
        mCpmOffset = par("cpmOffset");
        mCpmInterval = par("cpmInterval");

        mStationType = StationType_passengerCar;

        std::string name=getParentModule()->getParentModule()->getFullName();
        std::string rsu="rsu";

        const auto cfg = getSimulation()->getEnvir()->getConfigEx();
        const auto cfg_run_number = cfg->getVariable("runnumber");
        runnumber= std::string(cfg_run_number);

        std::ofstream myfile;
        // myfile.open("atest.out",  std::ios_base::app);
        

        if (name.find(rsu) != std::string::npos) {
            mStationType = StationType_roadSideUnit;
            mGeoPosition = &getFacilities().get_const<artery::GeoPosition>();
            // myfile<<"rsu\n";
            mFixedRate=true;
            // if(mFixedRate){
            //     myfile<<"fixed\n";
            // }
        }
        else{
            mVehicleDataProvider = &getFacilities().get_const<artery::VehicleDataProvider>();
            _mVehicleController = &getFacilities().get_const<traci::VehicleController>();
            // myfile<<"car\n";



        }

        // myfile.close();

        if (name.find(rsu) == std::string::npos) {
            
            myfile.open("results/results_"+runnumber+"/uniqueids.out",  std::ios_base::app);
            myfile << mVehicleDataProvider->station_id()<<"\n";
            myfile.close();
        }

        mTimer = &getFacilities().get_const<artery::Timer>();
        localEnvmod = &getFacilities().get_const<artery::LocalEnvironmentModel>();
        mPrimaryChannel = getFacilities().get_const<artery::MultiChannelPolicy>().primaryChannel(vanetza::aid::CP);
        mNetworkInterfaceTable = &getFacilities().get_const<artery::NetworkInterfaceTable>();
        mIdentityRegistry = inet::findModuleFromPar<artery::IdentityRegistry>(par("identityRegistryModule"), getParentModule()->getParentModule()->getParentModule());

    } else if (stage == artery::InitStages::Self) {
        mHostId = getFacilities().get_const<artery::Identity>().host->getId();
    } else if (stage == artery::InitStages::Propagate) {
        for (const artery::Sensor* sensor : mEnvironmentModel->getSensors()) {
            // consider only sensors with a field of view
            if (auto fovSensor = dynamic_cast<const artery::FovSensor*>(sensor)) {
                artery::CollectivePerceptionMockMessage::FovContainer fovContainer;
                fovContainer.sensorId = sensor->getId();
                fovContainer.position = sensor->position();
                fovContainer.fov = fovSensor->getFieldOfView();
                mFovContainers.emplace_back(std::move(fovContainer));
                mSensors.insert(sensor);
            }
        }
        if(mStationType == StationType_passengerCar){
            std::ofstream myfile;
            myfile.open("results/vehicles.out",  std::ios_base::app);
            myfile<<mHostId<<"\n";

            myfile.close();
        }
        else{
            std::ofstream myfile;
            std::string name=getParentModule()->getParentModule()->getFullName();
            myfile.open("results/vehicles.out",  std::ios_base::app);
            myfile<<name<<"\n";

            myfile.close();
        }
    }
}

void CollectivePerceptionService::handleMessage(omnetpp::cMessage* msg)
{
    if (msg == mTrigger) {
        generatePacket();
    } else {
        ItsG5BaseService::handleMessage(msg);
    }
}

void CollectivePerceptionService::trigger()
{
    Enter_Method("trigger");

    auto T_now = simTime();

    SimTime& T_GenCpm = mGenCpm;
	const SimTime& T_GenCpmMin = mGenCpmMin;
	const SimTime& T_GenCpmMax = mGenCpmMax;
	const SimTime T_GenCpmDcc = mDccRestriction ? genCpmDcc() : T_GenCpmMin;
	const SimTime T_elapsed = T_now - mLastCpmTimestamp;



    if (T_elapsed >= T_GenCpmDcc) {
        if(mStationType == StationType_passengerCar){
            if (mFixedRate) {
                generatePacket();
            } else if (checkHeadingDelta() || checkPositionDelta() || checkSpeedDelta()) {
                generatePacket();
                T_GenCpm = std::min(T_elapsed, T_GenCpmMax); /*< if middleware update interval is too long */
                mGenCpmLowDynamicsCounter = 0;
            } else if (T_elapsed >= T_GenCpm) {
                generatePacket();
                if (++mGenCpmLowDynamicsCounter >= mGenCpmLowDynamicsLimit) {
                    T_GenCpm = T_GenCpmMax;
                }
            }
        }
        else{
            if (mFixedRate) {
                generatePacket();
            } else if (T_elapsed >= T_GenCpm) {
                generatePacket();
                if (++mGenCpmLowDynamicsCounter >= mGenCpmLowDynamicsLimit) {
                    T_GenCpm = T_GenCpmMax;
                }
            }
        }
	}

}

bool CollectivePerceptionService::checkHeadingDelta() const
{
	return !vanetza::facilities::similar_heading(mLastCpmHeading, mVehicleDataProvider->heading(), mHeadingDelta);
}

bool CollectivePerceptionService::checkPositionDelta() const
{
	return (distance(mLastCpmPosition, mVehicleDataProvider->position()) > mPositionDelta);
}

bool CollectivePerceptionService::checkSpeedDelta() const
{
	return abs(mLastCpmSpeed - mVehicleDataProvider->speed()) > mSpeedDelta;
}


omnetpp::SimTime CollectivePerceptionService::genCpmDcc()
{
	// network interface may not be ready yet during initialization, so look it up at this later point
	auto netifc = mNetworkInterfaceTable->select(mPrimaryChannel);
	vanetza::dcc::TransmitRateThrottle* trc = netifc ? netifc->getDccEntity().getTransmitRateThrottle() : nullptr;
	if (!trc) {
		throw cRuntimeError("No DCC TRC found for CA's primary channel %i", mPrimaryChannel);
	}

	static const vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP2, 0);
	vanetza::Clock::duration interval = trc->interval(ca_tx);
	SimTime dcc { std::chrono::duration_cast<std::chrono::milliseconds>(interval).count(), SIMTIME_MS };
	return std::min(mGenCpmMax, std::max(mGenCpmMin, dcc));
}

void CollectivePerceptionService::indicate(const vanetza::btp::DataIndication& ind, std::unique_ptr<vanetza::UpPacket> packet)
{
    Enter_Method("indicate");

    artery::Asn1PacketVisitor<vanetza::asn1::Cpm> visitor;
	const vanetza::asn1::Cpm* cpm = boost::apply_visitor(visitor, *packet);
    
    
	if (cpm && cpm->validate()) {
        
		CpObject obj = visitor.shared_wrapper;
        // std::cout<<mHostId<< " get data:"<<obj.asn1()->header.stationID <<" time:"<<omnetpp::simTime()<<" gendeltatime: "<<obj.asn1()->cpm.generationDeltaTime <<"\n";
		emit(cpmReceivedSignal, &obj);
		// mLocalDynamicMap->updateAwareness(obj);
	}
}

void CollectivePerceptionService::generatePacket()
{
    uint16_t genDeltaTimeMod;
    vanetza::asn1::Cpm cpm;
    if(mStationType == StationType_passengerCar){
        genDeltaTimeMod = artery::countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
        cpm = createCollectivePerceptionMessage(*mVehicleDataProvider, genDeltaTimeMod);
        if( cpm->cpm.cpmParameters.numberOfPerceivedObjects==0 ) return;

        mLastCpmPosition = mVehicleDataProvider->position();
        mLastCpmSpeed = mVehicleDataProvider->speed();
        mLastCpmHeading = mVehicleDataProvider->heading();

    }
    else{
        genDeltaTimeMod = 0;
        // genDeltaTimeMod =  artery::countTaiMilliseconds(mTimer->getCurrentTime());
        cpm = createCollectivePerceptionMessage(*mVehicleDataProvider, genDeltaTimeMod);
        if( cpm->cpm.cpmParameters.numberOfPerceivedObjects==0 ) return;
    }
    

    

    using namespace vanetza;
	btp::DataRequestB request;
	request.destination_port = btp::ports::CPM;
	request.gn.its_aid = aid::CP;
	request.gn.transport_type = geonet::TransportType::SHB;
	request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
	request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
	request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

	CpObject obj(std::move(cpm));
    mLastCpmTimestamp = omnetpp::simTime();
	emit(cpmSentSignal, &obj);

    // std::cout<<"send data:"<<mHostId<<" time:"<<omnetpp::simTime()<<" gendeltatime: "<<genDeltaTimeMod <<"\n";

    std::ofstream myfile;
    // myfile.open("results/tests.out",  std::ios_base::app);

	using CpmByteBuffer = convertible::byte_buffer_impl<asn1::Cpm>;
	std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
	std::unique_ptr<convertible::byte_buffer> buffer { new CpmByteBuffer(obj.shared_ptr()) };
	payload->layer(OsiLayer::Application) = std::move(buffer);
	this->request(request, std::move(payload));

}

vanetza::asn1::Cpm CollectivePerceptionService::createCollectivePerceptionMessage(const artery::VehicleDataProvider& vdp, uint16_t genDeltaTime){
    // std::cout<<"entered create message\n";
    
    vanetza::asn1::Cpm message;

    ItsPduHeader_t& header = (*message).header;
	header.protocolVersion = 2;
	header.messageID = ItsPduHeader__messageID_cpm;
	header.stationID = mHostId;

    bool gottime=false;
    int i=0;

	// BasicVehicleContainerLowFrequency& bvc = lfc->choice.basicVehicleContainerLowFrequency;

    using TrackedObject = artery::LocalEnvironmentModel::TrackedObject;
    // const TraCIAPI::SimulationScope* m_sim=&(obj->getController()->getTraCI()->simulation);// = _mVehicleController->getTraCI()->simulation;

    libsumo::TraCIPosition vehpos;

    uint16_t genDeltaTimeMod;
    
    for (const auto& object : localEnvmod->allObjects()) {
        const artery::LocalEnvironmentModel::Tracking& tracking = object.second;
        if (tracking.expired()) {
            // skip objects with lost tracking
            continue;
        }

        // for (const auto& sensor : tracking.sensors()) {
        std::shared_ptr<artery::EnvironmentModelObject> obj=object.first.lock();
        if(!obj){
            continue;
        }
        else{
            const artery::VehicleDataProvider* objDataProvider=&(obj->getVehicleData());
            genDeltaTimeMod = artery::countTaiMilliseconds(mTimer->getTimeFor(objDataProvider->updated()));
            gottime=true;
        }
        if(gottime) break;
        // }
        if(gottime) break;
    }

    CollectivePerceptionMessage_t& cpm = (*message).cpm;
    cpm.generationDeltaTime = genDeltaTimeMod * GenerationDeltaTime_oneMilliSec;
    CpmManagementContainer_t& basic = cpm.cpmParameters.managementContainer;
    cpm.cpmParameters.numberOfPerceivedObjects=0;

    basic.stationType = mStationType;
	basic.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
	basic.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
    if(&vdp != nullptr){
        basic.referencePosition.longitude = round(vdp.longitude(), microdegree6) * Longitude_oneMicrodegreeEast;
        basic.referencePosition.latitude = round(vdp.latitude(), microdegree6) * Latitude_oneMicrodegreeNorth;
    }
    else{
        basic.referencePosition.longitude = round(mGeoPosition->longitude, microdegree6) * Longitude_oneMicrodegreeEast;
        basic.referencePosition.latitude = round(mGeoPosition->latitude, microdegree6) * Latitude_oneMicrodegreeNorth;
    }
	basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
			SemiAxisLength_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
			SemiAxisLength_unavailable;


    addPerceptionObjects(message);

    std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid CPM: %s", error.c_str());
	}

    // std::cout<<message.validate(error)<<" created message "<<error << "\n";

    return message;
}

int getdelta(double posa, double posb){

    int a=(int)(posa*10);
    int b=(int)(posb*10);

    // std::cout<<"posb:"<<posb<<" posa:"<< (posa)<<"\n";
    // std::cout<<"b:"<<b<<" a:"<< (a)<<"\n";
    // int test=a-b;

    // std::cout<<"b:"<<b<<" bd:"<< (a-test)<<"\n";

    // int delta;
    // if(a>b){
    //     delta=abs(a-b);
    // }
    // else delta=abs(b-a);

    // int testval = delta+a;
    // // if(a<-10 || b <-10)
    // //     std::cout<<"delta calc "<<"posb:"<<b<<" re:"<< testval<<"\n";


    // if(testval!=b){
    //     std::cout<<"delta calc "<<"posb:"<<b<<" re:"<< testval<<"\n";
    //     delta *=-1;
    //     std::cout<<"after delta calc "<<"posb:"<<b<<" re:"<< testval<<"\n";
    // }
    // if((delta+posa)!=posb)
    //     std::cout<<"after delta calc "<<"posb:"<<std::format("{}", posb)<<" re:"<< std::format("{}", (delta+posa))<<"\n";
    return a-b;
}

void CollectivePerceptionService::addPerceptionObjects(vanetza::asn1::Cpm& message)
{
	PerceivedObjectContainer_t*& poc = message->cpm.cpmParameters.perceivedObjectContainer;
    poc = vanetza::asn1::allocate<PerceivedObjectContainer_t>();

	// BasicVehicleContainerLowFrequency& bvc = lfc->choice.basicVehicleContainerLowFrequency;

    using TrackedObject = artery::LocalEnvironmentModel::TrackedObject;
    // const TraCIAPI::SimulationScope* m_sim=&(obj->getController()->getTraCI()->simulation);// = _mVehicleController->getTraCI()->simulation;

    libsumo::TraCIPosition vehpos;// = m_sim->convertGeo(mVehicleDataProvider->latitude().value(), mVehicleDataProvider->longitude().value(), true);

    bool notset=true;
    int i=0;

    uint16_t genDeltaTimeMod;
    
    for (const auto& object : localEnvmod->allObjects()) {
        const artery::LocalEnvironmentModel::Tracking& tracking = object.second;
        if (tracking.expired()) {
            // skip objects with lost tracking
            continue;
        }

        // for (const auto& sensor : tracking.sensors()) {

        if(poc->list.count>=50){
            break;
        }

        std::shared_ptr<artery::EnvironmentModelObject> obj=object.first.lock();

        if(!obj){
            continue;
        }
        else{
            const artery::VehicleDataProvider* objDataProvider=&(obj->getVehicleData());

            auto m_sim=&(obj->getController()->getTraCI()->simulation);// = _mVehicleController->getTraCI()->simulation;
            
            if(mStationType == StationType_passengerCar){
                vehpos = m_sim->convertGeo(mVehicleDataProvider->latitude().value(), mVehicleDataProvider->longitude().value(), true);
            }
            else{
                vehpos = m_sim->convertGeo(mGeoPosition->latitude.value(), mGeoPosition->longitude.value(), true);
            }

            PerceivedObject* pobject = vanetza::asn1::allocate<PerceivedObject>();

            auto objpos = m_sim->convertGeo(objDataProvider->latitude().value(), objDataProvider->longitude().value(), true);

            int idval =getObjectID(objDataProvider->station_id());

            if(idval==-1){
                continue;
            }

            if( sqrt(pow(objpos.x-vehpos.x, 2) + pow(objpos.y-vehpos.y, 2) * 1.0) > mSensorRange)
                continue;

            std::ofstream myfile;
            myfile.open("results/results_"+runnumber+"/uniqueids.out",  std::ios_base::app);
            myfile << objDataProvider->station_id()<<"\n";
            myfile.close();

            pobject->objectID = idval;
            i++;
            pobject->timeOfMeasurement = 0;
            pobject->objectConfidence = 95;
            pobject->xDistance.confidence = 95;
            int xdelta = getdelta(vehpos.x, objpos.x);
            pobject->xDistance.value = xdelta;
            pobject->yDistance.confidence = 95;
            int ydelta = getdelta(vehpos.y, objpos.y);

            pobject->yDistance.value = ydelta;

            // std::cout<<"x:"<<objpos.x<<" dx:"<<(xdelta+vehpos.x) <<" y:"<<objpos.y<<" dx:"<<(ydelta+vehpos.y)<<"\n";

            pobject->xSpeed.confidence = (long) SpeedConfidence::SpeedConfidence_equalOrWithinOneMeterPerSec;
            pobject->xSpeed.value = floor(objDataProvider->speed().value()*100);
            // std::cout<<pobject->xSpeed.value <<"\n";

            pobject->ySpeed.confidence = (long) SpeedConfidence::SpeedConfidence_equalOrWithinOneMeterPerSec;
            pobject->ySpeed.value = floor(objDataProvider->speed().value()*100);

            pobject->yawAngle = vanetza::asn1::allocate<CartesianAngle>();

            pobject->yawAngle->confidence = AngleConfidence::AngleConfidence_oneDegree;
            pobject->yawAngle->value = floor(10*objDataProvider->heading().value()*180.0/PI);

            
            int axval = floor( objDataProvider->acceleration().value()*10);

            if(axval>160){
                axval=159;
            }
            if(axval<-160){
                axval=-159;
            }

            pobject->xAcceleration = vanetza::asn1::allocate<LongitudinalAcceleration>();

            pobject->xAcceleration->longitudinalAccelerationConfidence=95;
            pobject->xAcceleration->longitudinalAccelerationValue = axval;

            pobject->yAcceleration = vanetza::asn1::allocate<LateralAcceleration>();

            pobject->yAcceleration->lateralAccelerationConfidence=95;
            pobject->yAcceleration->lateralAccelerationValue = floor( objDataProvider->yaw_rate().value()*10);


            ASN_SEQUENCE_ADD(&poc->list, pobject);

        }

        // }
    }

    message->cpm.cpmParameters.numberOfPerceivedObjects=poc->list.count;

    // std::cout<<"Message size: "<<message.size()<<" object count: "<<poc->list.count<<"\n";

    
	

	std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid CPM Perception Container: %s", error.c_str());
	}
}

int CollectivePerceptionService::getObjectID(int badid){

    // std::ofstream myfile;
    // myfile.open("results/test.out",  std::ios_base::app);
    int cor=1;
    bool found=false;

    std::string teststr= "vehicle_"+std::to_string(cor);
    while(cor<60){
        teststr= "vehicle_"+std::to_string(cor);
        // myfile<<teststr<<"\n";

        if (mIdentityRegistry) {
            auto identity = mIdentityRegistry->lookup<artery::IdentityRegistry::traci>(teststr);
            if (identity) {
                // myfile<<teststr<<" foundid: "<<identity->application<<"\n";

                if(identity->application==badid){
                    found=true;
                    return cor;
                }


                // myfile.close();
                // return identity->application;
            }
        }
        cor++;
    }

    // myfile.close();
    return -1;
}
