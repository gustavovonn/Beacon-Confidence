#include "MonitorCollectivePerception.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/CaObject.h"
#include "motion_vector.h"
#include <math.h> 
#include <fstream>
#include <boost/math/constants/constants.hpp>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/checkandcast.h>
#include <inet/common/ModuleAccess.h>
#include <omnetpp/cconfiguration.h>
#include <sys/stat.h>

#define BOOST_GEOMETRY_DISABLE_DEPRECATED_03_WARNING
#define MAXLINE 1024
#define FLOAT_INT_PRECISION 100000000. //# float int precision for longitude and latitude in LLA (rad) --> 1e8 because 1e9 would overflow for values close to -180 or 180.
#define FLOAT_INT_PRECISION_M 1000. //# float int precision for altitude in LLA (meters)

auto microdegree4 = vanetza::units::degree * boost::units::si::micro;

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}


using namespace omnetpp;

static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");
# define M_PI           3.14159265358979323846  /* pi */

Define_Module(MonitorCollectivePerception)

void MonitorCollectivePerception::initialize(int stage) {
    CollectivePerceptionService::initialize(stage);
    _lpn = &getFacilities().get_mutable<LocalPerceptionNode>();
    globalEnvmod = &getFacilities().get_mutable<artery::GlobalEnvironmentModel>();
    _mVehicleController = &getFacilities().get_const<traci::VehicleController>();
    _mVehicleDataProvider = &getFacilities().get_const<artery::VehicleDataProvider>();
    mTimer = &getFacilities().get_const<artery::Timer>();
    mIdentityRegistry = inet::findModuleFromPar<artery::IdentityRegistry>(par("identityRegistryModule"), getParentModule()->getParentModule()->getParentModule());
    
    const auto cfg = getSimulation()->getEnvir()->getConfigEx();
    const auto cfg_run_number = cfg->getVariable("runnumber");
    runnumber= std::string(cfg_run_number);

    smartdataEnabled = par("smartdataEnabled");

    if(smartdataEnabled){
        sdip = "224.1.1.1";
        sdport = 5000+1+28;
        // _MULTICAST_TTL = 2

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

    std::string folder("results/results_"+runnumber);

    const int dir_err = mkdir(folder.c_str() , S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    
    std::ofstream myfile;
    myfile.open("results/results_"+runnumber+"/monitor_CPM-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::trunc);
    // myfile<<getParentModule()->getParentModule()->getParentModule()->getFullName();
    myfile.close();
}

void MonitorCollectivePerception::sendsd(unsigned char * data, int size){
    sendto(sockfd, (const char *)data, size, 
            MSG_CONFIRM, (const struct sockaddr *) &servaddr,  
                sizeof(servaddr)); 
    // std::cout<<"Hello message sent."<<std::endl; 
}

std::string MonitorCollectivePerception::receivesd(){
    char buffer[MAXLINE];
    int n;
    socklen_t len;

    n = recvfrom(sockfd, (char *)buffer, MAXLINE,  
                MSG_WAITALL, (struct sockaddr *) &servaddr, 
                &len); 
    buffer[n] = '\0'; 
    std::cout<<"Server :"<<buffer<<std::endl; 

    std::string retstr(buffer);

    return retstr;
}

void MonitorCollectivePerception::trigger(){

}

void MonitorCollectivePerception::indicate(const vanetza::btp::DataIndication& ind, std::unique_ptr<vanetza::UpPacket> packet)
{
    Enter_Method("indicate");

    artery::Asn1PacketVisitor<vanetza::asn1::Cpm> visitor;
	const vanetza::asn1::Cpm* cpm = boost::apply_visitor(visitor, *packet);

    std::vector<unsigned char> mvvec;
    
	if (cpm && cpm->validate()) {

        auto msgts=mTimer->getTimeFor(mTimer->reconstructMilliseconds( (*cpm)->cpm.generationDeltaTime )).inUnit(omnetpp::SIMTIME_US);

        auto thists=_mVehicleDataProvider->updated().inUnit(omnetpp::SIMTIME_US);
        // if(msgts != thists){
        std::ofstream myfile;
        myfile.open("results/results_"+runnumber+"/oldcpm-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
        myfile<<thists<<" "<<msgts<<"\n";
        myfile.close();
        // }

        auto& m_sim = _mVehicleController->getTraCI()->simulation;
        
		CpObject obj = visitor.shared_wrapper;

        // std::ofstream myfile;
        myfile.open("results/results_"+runnumber+"/monitor_CPM-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);

        double originlat = (*cpm)->cpm.cpmParameters.managementContainer.referencePosition.latitude/10000000.0;
        double originlon = (*cpm)->cpm.cpmParameters.managementContainer.referencePosition.longitude/10000000.0;

        auto vehpos = m_sim.convertGeo(originlat, originlon, true);

        auto numberofObjects = (*cpm)->cpm.cpmParameters.numberOfPerceivedObjects;

        auto objarr = (*cpm)->cpm.cpmParameters.perceivedObjectContainer->list.array;
        // myfile<<"or lat: "<<(*cpm)->cpm.cpmParameters.managementContainer.referencePosition.latitude/10000000.0<<" lon: "<<(*cpm)->cpm.cpmParameters.managementContainer.referencePosition.longitude/10000000.0<<"\n";

        // myfile<<"lat: "<<originlat<<" lon: "<<originlon<<"\n";
        // myfile<<"vehposx: "<<vehpos.x<<" vehposy: "<<vehpos.y<<"\n";

        // int cor=1;
        // bool found=false;
        // std::string teststr= "vehicle_"+std::to_string(cor);
        // while(true){
        //     teststr= "vehicle_"+std::to_string(cor);
        //     // myfile<<teststr<<"\n";
        //     auto test = globalEnvmod->getObject(teststr);
        //     const artery::VehicleDataProvider* OrigobjDataProvider=&(test->getVehicleData());
        //     if(test!=nullptr){
        //         // myfile<<"in: "<<cor<<"\n";
        //         if(OrigobjDataProvider->station_id()==(*cpm)->header.stationID){
        //             found=true;
        //             break;
        //         }
        //     }
        //     else{
        //         // myfile<<"out: "<<cor<<"\n";
        //         break;
        //     }
        //     cor++;
        // }
        // const artery::VehicleDataProvider* OrigobjDataProvider=nullptr;
        // if(found){

        //     auto test = globalEnvmod->getObject(teststr);
        //     // myfile<<"test: "<<globalEnvmod->mObjects.begin()->first<<"\n";

        //     if(test!=nullptr){

        //         OrigobjDataProvider=&(test->getVehicleData());
        //         // newdata.objDataProvider=objDataProvider;
        //         // myfile<<OrigobjDataProvider->getStationId()<<"\n";
        //         // if(_lpn!=nullptr)
        //         // newdata.objDataProvider=nullptr;
        //         // _lpn->addDataCPM(newdata, OrigobjDataProvider);

        //     }
        // }

        // myfile<<"got "<<numberofObjects<<" objects in message: \n";

        int size = 30;
        unsigned char temp = '\x00';
        if (size > numberofObjects)
            size = numberofObjects;
        unsigned long long tempid = _mVehicleDataProvider->station_id();
        for(int i=0; i<size; i++){
            // myfile<<i<<" id: "<<objarr[i]->objectID<<"\n";
            Motion_Vector_Global s;

            auto posx = vehpos.x-objarr[i]->xDistance.value/10.0;
            auto posy = vehpos.y-objarr[i]->yDistance.value/10.0;

            // if(posx<0 || posy<0)
            // std::cout<<"posx: "<<posx<<" posy: "<<posy<<" dist "<<sqrt(pow(posx-vehpos.x, 2) + pow(posy-vehpos.y, 2) * 1.0)<<"\n";

            auto vehpos = m_sim.convertGeo(posx, posy, false);

            // myfile<<"poslat: "<<vehpos.x<<" poslon: "<<vehpos.y<<"\n";

            // myfile<<"speed: "<<objarr[i]->xSpeed.value/100.0<<"\n";

            // myfile<<"heading: "<<objarr[i]->yawAngle->value/10.0<<"\n";

            s._id = objarr[i]->objectID;

            s._location._lat._value = round(vehpos.x*boost::units::degree::degree, microdegree4) * Longitude_oneMicrodegreeEast * (M_PI/180.0); // Latitude_oneMicrodegreeNorth = 10 --> positive
            s._location._lat._uncertainty = 0; // TODO: from elipse to uncertainty
            s._location._lon._value = round(vehpos.y*boost::units::degree::degree, microdegree4) * Latitude_oneMicrodegreeNorth * (M_PI/180.0);
            s._location._lon._uncertainty = 0;
            s._location._alt._value = 0; // AltitudeValue_unavailable
            s._location._alt._uncertainty = 100;
            s._speed._value = objarr[i]->xSpeed.value/100.0;
            s._speed._uncertainty = 0;
            s._heading._value = objarr[i]->yawAngle->value/10.0 * (M_PI/180.0);   // heading is already in radians
            // std::cout<<"cp: "<<s._heading._value<<"\n";
            s._heading._uncertainty = 0;
            s._class = 5; // StationType_passengerCar
            s._uncertainty = 0;
            s._yaw_rate._value = objarr[i]->yAcceleration->lateralAccelerationValue/10.0; // rad/s
            s._yaw_rate._uncertainty = 0;
            s._acceleration._value = objarr[i]->xAcceleration->longitudinalAccelerationValue/10.0; // / vanetza::units::si::meter_per_second_squared;
            s._acceleration._uncertainty = 0;
            s._timestamp = mTimer->getTimeFor(mTimer->reconstructMilliseconds( (*cpm)->cpm.generationDeltaTime )).inUnit(omnetpp::SIMTIME_US);
            object_pair newdata;
            newdata.mv=s;
            myfile.close();
            std::string teststr= "vehicle_"+std::to_string(s._id);
            // int testid = getObjectID(s._id);

            auto identity = mIdentityRegistry->lookup<artery::IdentityRegistry::traci>(teststr);
            if (identity) {

                // if(OrigobjDataProvider){
                    // newdata.objDataProvider=objDataProvider;
                    // myfile<<OrigobjDataProvider->getStationId()<<"\n";
                    // if(_lpn!=nullptr)
                    newdata.mv._id=identity->application;
                    s._id=identity->application;
                    // newdata.objDataProvider=nullptr;
                    _lpn->addDataCPM(newdata, vehpos);
                    myfile.open("results/results_"+runnumber+"/monitor_CPM-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
                    myfile << s.to_string();
                    myfile.close();

                    if(smartdataEnabled){

                        unsigned char data[53];

                        data[0]='\x01';

                        // location=[int(lon*FLOAT_INT_PRECISION), int(lat*FLOAT_INT_PRECISION), int(alt*FLOAT_INT_PRECISION_M)],
                        
                        int lon = (int) (newdata.mv._location._lon._value*100);
                        memcpy(&data[1], reinterpret_cast<unsigned char *> (&lon), sizeof(int) );
                        int lat = (int) (newdata.mv._location._lat._value*100);
                        memcpy(&data[5], reinterpret_cast<unsigned char *> (&lat), sizeof(int) );
                        int alt = (int) (newdata.mv._location._alt._value*FLOAT_INT_PRECISION_M);
                        memcpy(&data[9], reinterpret_cast<unsigned char *> (&alt), sizeof(int) );

                        //speed
                        memcpy(&data[13], reinterpret_cast<unsigned char *> (&newdata.mv._speed._value), sizeof(float) );
                        //heading
                        memcpy(&data[17], reinterpret_cast<unsigned char *> (&newdata.mv._heading._value), sizeof(float) );
                        //yaw rate
                        memcpy(&data[21], reinterpret_cast<unsigned char *> (&newdata.mv._yaw_rate._value), sizeof(float) );

                        // acceleration
                        memcpy(&data[25], reinterpret_cast<unsigned char *> (&newdata.mv._acceleration._value), sizeof(float) );
                        //ID
                        // std::cout<<"ID: "<<newdata.mv._id<<"\n";
                        unsigned long long tempid= newdata.mv._id;
                        memcpy(&data[29], reinterpret_cast<unsigned char *> (&tempid), sizeof(unsigned long long) );
                    
                        int temp=0;
                        //uncertainty
                        memcpy(&data[37], reinterpret_cast<unsigned char *> (&temp), sizeof(float) );

                        temp=12;
                        //class: 'Reference'  : 12
                        memcpy(&data[41], reinterpret_cast<unsigned char *> (&temp), sizeof(float) );

                        //timestamp
                        memcpy(&data[45], reinterpret_cast<unsigned char *> (&newdata.mv._timestamp), sizeof(unsigned long long) );

                        // sendsd(data, 54);
                        for(int i=0; i<53;i++){
                            mvvec.push_back(data[i]);
                        }
                        
                        
                    }

                // }

            }

            // myfile.open("results/monitor_CPM.out",  std::ios_base::app);
            // myfile<<"newid: "<<testid<<"\n";

        }
        if (size < 30) {
            for (int i = size; i < 30; i++)
            {
                for(int j=0; j<53;j++){ // fill in blank to keep consistency of size
                    mvvec.push_back(temp);
                }
            }
        }
        mvvec.push_back(temp); // ending of smartdata packet (e.g., next is invalid) (nx53)+1
        // add signature of vehicle producing data
        for(int i=0; i<8;i++) {
            mvvec.push_back(reinterpret_cast<unsigned char *>(&tempid)[i]);
        }

        // mvvec.data()
        // myfile<<"\n";

        // CollectivePerceptionMessage_t data = &(obj).asn1()->cpm;

	}
    sendsd(mvvec.data(), mvvec.size());
}

int MonitorCollectivePerception::getObjectID(int badid){

    std::ofstream myfile;
    myfile.open("results/results_"+runnumber+"/monitor_CPM-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
    int cor=1;
    bool found=false;


    std::string teststr= "vehicle_"+std::to_string(cor);
    while(cor<60){
        teststr= "vehicle_"+std::to_string(cor);
        // myfile<<teststr<<"\n";

        if (mIdentityRegistry) {
            auto identity = mIdentityRegistry->lookup<artery::IdentityRegistry::traci>(teststr);
            if (identity) {
                // myfile<<"foundid: "<<identity->application<<"\n";

                for(int i=badid; i<8000; i+=241){
                    if(identity->application==i){
                        found=true;
                        return i;
                        break;
                    }
                    if(i>identity->application) break;
                }

                // myfile.close();
                // return identity->application;
            }
        }
        cor++;
    }

    myfile.close();
    return -1;
}

    // myfile << "am here\n";

    // const std::vector<artery::CollectivePerceptionMockMessage::ObjectContainer>& objContainer = cpm->getObjectContainers();

    // myfile<<"size:"<<objContainer.size()<<"\n";

    // for(auto it = objContainer.begin(); it != objContainer.end(); ++it) {
    // // for (const auto& object : objContainer) {
    //     std::shared_ptr<artery::EnvironmentModelObject> obj = it->object.lock();

    //     if(!obj){
    //         continue;
    //     }
    //     else{

    //         const artery::VehicleDataProvider* objDataProvider=&(obj->getVehicleData());

            // Motion_Vector_Global s;

            // s._id = objDataProvider->station_id();;
            // s._location._lat._value = round(objDataProvider->latitude(), microdegree4) * Longitude_oneMicrodegreeEast * (M_PI/180.0); // Latitude_oneMicrodegreeNorth = 10 --> positive
            // s._location._lat._uncertainty = 0; // TODO: from elipse to uncertainty
            // s._location._lon._value = round(objDataProvider->longitude(), microdegree4) * Latitude_oneMicrodegreeNorth * (M_PI/180.0);
            // s._location._lon._uncertainty = 0;
            // s._location._alt._value = 0; // AltitudeValue_unavailable
            // s._location._alt._uncertainty = 100;
            // s._speed._value = objDataProvider->speed().value();
            // s._speed._uncertainty = 0;
            // s._heading._value = objDataProvider->heading().value();   // heading is already in radians
            // s._heading._uncertainty = 0;
            // s._yaw_rate._value = objDataProvider->yaw_rate().value(); // rad/s
            // s._yaw_rate._uncertainty = 0;
            // s._acceleration._value = objDataProvider->acceleration().value(); // / vanetza::units::si::meter_per_second_squared;
            // s._acceleration._uncertainty = 0;
            // s._dimension._length._value = obj->getLength().value();
            // s._dimension._length._uncertainty = 0;
            // s._dimension._width._value = obj->getWidth().value();
            // s._dimension._width._uncertainty = 0;
            // s._dimension._height._value = 0;
            // s._dimension._height._value = 100;
            // s._class = 5; // StationType_passengerCar
            // s._uncertainty = 0;
            // s._timestamp = objDataProvider->updated().inUnit(omnetpp::SIMTIME_US);

    //         // myfile << s.to_string();

    //         object_pair newdata;
    //         newdata.mv=s;
    //         newdata.objDataProvider=objDataProvider;
    //         // myfile<<"src: "<<cpm->getSourceStation()<<"\n";

    //         int cor=1;
    //         bool found=false;
    //         std::string teststr= "vehicle_"+std::to_string(cor);
    //         while(true){
    //             teststr= "vehicle_"+std::to_string(cor);
    //             // myfile<<teststr<<"\n";
    //             auto test = globalEnvmod->getObject(teststr);
    //             const artery::VehicleDataProvider* OrigobjDataProvider=&(test->getVehicleData());
    //             if(test!=nullptr){
    //                 // myfile<<"in: "<<cor<<"\n";
    //                 if(OrigobjDataProvider->station_id()==cpm->getSourceStation()){
    //                     found=true;
    //                     break;
    //                 }
    //             }
    //             else{
    //                 // myfile<<"out: "<<cor<<"\n";
    //                 break;
    //             }
    //             cor++;
    //         }
    //         if(found){

    //             auto test = globalEnvmod->getObject(teststr);
    //             // myfile<<"test: "<<globalEnvmod->mObjects.begin()->first<<"\n";

    //             if(test!=nullptr){
    //                 const artery::VehicleDataProvider* OrigobjDataProvider=&(test->getVehicleData());
    //                 // newdata.objDataProvider=objDataProvider;
    //                 // myfile<<mVehicleDataProvider->getStationId()<<"\n";
    //                 // if(_lpn!=nullptr)
    //                 _lpn->addDataCPM(newdata, OrigobjDataProvider);

    //             }
    //             // test.get
    //             // test.
    //             // if(test)
    //             // newdata.objDataProvider=&test->getVehicleData();
    //             // myfile<<test->getVehicleData();
    //             // myfile<<"test "<<newdata.objDataProvider->station_id()<<"\n";

    //             // myfile.close();

                
    //         }

    //         // _lpn->addDataLocal(newdata);
    //     }
    // }



    // myfile.close();
    // emit(cpmReceivedSignal, cpm);

// }
