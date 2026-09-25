#include "MonitorCa.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/CaObject.h"
#include "motion_vector.h"
#include <math.h> 
#include <fstream>
#include <boost/math/constants/constants.hpp>
#include <boost/units/cmath.hpp>
#include <omnetpp/cconfiguration.h>
#include <sys/stat.h>

#define BOOST_GEOMETRY_DISABLE_DEPRECATED_03_WARNING
#define MAXLINE 1024
#define FLOAT_INT_PRECISION 100000000. //# float int precision for longitude and latitude in LLA (rad) --> 1e8 because 1e9 would overflow for values close to -180 or 180.
#define FLOAT_INT_PRECISION_M 1000. //# float int precision for altitude in LLA (meters)

using namespace omnetpp;

static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");
# define M_PI           3.14159265358979323846  /* pi */

Define_Module(MonitorCa)

void MonitorCa::initialize() {
    CaService::initialize();
    _socket = new UDP_Socket(SD_DEVICE-1, false);
    mTimer = &getFacilities().get_const<artery::Timer>();
    mVehicleDataProvider = &getFacilities().get_const<artery::VehicleDataProvider>();
    _mVehicleController = &getFacilities().get_const<traci::VehicleController>();
    globalEnvmod = &getFacilities().get_mutable<artery::GlobalEnvironmentModel>();

    const auto cfg = getSimulation()->getEnvir()->getConfigEx();
    const auto cfg_run_number = cfg->getVariable("runnumber");
    runnumber= std::string(cfg_run_number);

    std::string folder("results/results_"+runnumber);

    smartdataEnabled = par("smartdataEnabled");

    if(smartdataEnabled){
        sdip = "224.1.1.1";
        sdport = 5000+1+26;
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

    const int dir_err = mkdir(folder.c_str() , S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    
    _lpn = &getFacilities().get_mutable<LocalPerceptionNode>();
    std::ofstream myfile;
    myfile.open("results/results_"+runnumber+"/monitor_v2x-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::trunc);
    myfile.close();
}

void MonitorCa::sendsd(unsigned char * data, int size){
    sendto(sockfd, (const char *)data, size, 
            MSG_CONFIRM, (const struct sockaddr *) &servaddr,  
                sizeof(servaddr)); 
    // std::cout<<"Hello message sent."<<std::endl; 
}

std::string MonitorCa::receivesd(){
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

void MonitorCa::trigger() {
    //CaService::trigger();
}

void MonitorCa::indicate(const vanetza::btp::DataIndication& ind, std::unique_ptr<vanetza::UpPacket> packet)
{

	Enter_Method("indicate");

	artery::Asn1PacketVisitor<vanetza::asn1::Cam> visitor;
	const vanetza::asn1::Cam* cam = boost::apply_visitor(visitor, *packet);
	if (cam && cam->validate()) {
        // std::cout<<_mVehicleController->getVehicleId()<<" got CAM\n";
		artery::CaObject obj = visitor.shared_wrapper;
		//emit(scSignalCamReceived, &obj); // given some hard to solve compilation errors, is this required?
        const vanetza::asn1::Cam& msg = obj.asn1();

        const BasicContainer_t& basic = msg->cam.camParameters.basicContainer;
        const BasicVehicleContainerHighFrequency& bvc = msg->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;

        Motion_Vector_Global mv;
        // int value=1000000.0;
        int value=1;
        // mv.store_geo_as_ecef(basic.referencePosition.latitude, basic.referencePosition.longitude, basic.referencePosition.altitude.altitudeValue);
        mv._location._lat._value = basic.referencePosition.latitude/value * (M_PI/180.0);  // microdegree to radians
        mv._location._lat._uncertainty = 0; // TODO: from elipse to uncertainty
        mv._location._lon._value = basic.referencePosition.longitude/value * (M_PI/180.0); // microdegree to radians
        mv._location._lon._uncertainty = 0;
        mv._location._alt._value = 0; // microdegree to radians
        mv._location._alt._uncertainty = basic.referencePosition.altitude.altitudeConfidence;
        mv._speed._value = bvc.speed.speedValue/100.0;    
        mv._speed._uncertainty = 0;// int(100*0.03/bvc.speed.speedValue); // SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3; --> 3/100 --> 0.03m/s
        mv._heading._value = bvc.heading.headingValue/10.0 * (M_PI/180.0); // decidegree to radiasn
        // std::cout<<"ca: "<<mv._heading._value<<" "<<bvc.heading.headingValue<<"\n";
        mv._heading._uncertainty = 0;//int(100*10/bvc.heading.headingValue); // HeadingConfidence_equalOrWithinOneDegree; == 10 decidegree
        mv._yaw_rate._value = bvc.yawRate.yawRateValue * (M_PI/180.0); // degree per second to radians per second
        mv._yaw_rate._uncertainty = 0; // bvc.yawRate.yawRateConfidence --> not set when generating CAM
        mv._acceleration._value = bvc.longitudinalAcceleration.longitudinalAccelerationValue;
        mv._acceleration._uncertainty = 0; // bvc.longitudinalAcceleration.longitudinalConfidence is set to unavailable 

        mv._dimension._length._value = bvc.vehicleLength.vehicleLengthValue;
        mv._dimension._length._uncertainty = 0; //bvc.vehicleLength.vehicleLengthConfidenceIndication
        mv._dimension._width._value = bvc.vehicleWidth;
        mv._dimension._width._uncertainty = 0; //bvc.vehicleLength.vehicleLengthConfidenceIndication
        mv._dimension._height._value = 0;
        mv._dimension._height._value = 100; mv._class = basic.stationType; // StationType_passengerCar
        
        mv._id = msg->header.stationID;
        mv._uncertainty = 100;
        mv._timestamp = mTimer->getTimeFor(mTimer->reconstructMilliseconds(msg->cam.generationDeltaTime)).inUnit(omnetpp::SIMTIME_US);

        object_pair newdata;
        newdata.mv=mv;
        std::ofstream myfile;
        // myfile.open("results/results_"+runnumber+"/monitor_v2x-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
        
        int cor=1;
        bool found=false;
        std::string teststr= "vehicle_"+std::to_string(cor);
        while(true){
            teststr= "vehicle_"+std::to_string(cor);
            // myfile<<teststr<<"\n";
            auto test = globalEnvmod->getObject(teststr);
            const artery::VehicleDataProvider* objDataProvider=&(test->getVehicleData());
            if(test!=nullptr){
                // myfile<<"in: "<<cor<<"\n";
                if(objDataProvider->station_id()==mv._id){
                    found=true;
                    break;
                }
            }
            else{
                // myfile<<"out: "<<cor<<"\n";
                break;
            }
            cor++;
        }
        found=true;
        if(found){
            // std::cout<<_mVehicleController->getVehicleId()<<" recorded CAM\n";
            myfile.open("results/results_"+runnumber+"/monitor_v2x-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
            myfile << mv.to_string();
            // std::cout<<mv.to_string();
            myfile.close();

            myfile.open("results/results_"+runnumber+"/monitor_v2x-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
            // myfile<<"found: "<<cor<<"\n";
            // myfile << getParentModule()->getParentModule()->getParentModule()->getSubmodule("node",cor)->getId() <<" "<<mv._id<<"\n";
            // myfile<<getParentModule()->getParentModule()->getParentModule()->getSubmodule("node",cor)->info();

            auto test = globalEnvmod->getObject(teststr);
            // myfile<<"test: "<<globalEnvmod->mObjects.begin()->first<<"\n";

            if(test!=nullptr){
                const artery::VehicleDataProvider* objDataProvider=&(test->getVehicleData());
                // newdata.objDataProvider=objDataProvider;
                // myfile<<mVehicleDataProvider->getStationId()<<"\n";
                // if(_lpn!=nullptr)
                _lpn->addDataCAM(newdata);

                if(smartdataEnabled){

                    unsigned char data[54+sizeof(unsigned long long)];

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

                    data[53]='\x00';
                    memcpy(&data[54], reinterpret_cast<unsigned char *> (&tempid), sizeof(unsigned long long) );

                    sendsd(data, 54+sizeof(unsigned long long));
                    
                }

                // myfile<<objDataProvider->station_id()<<"\n";
                if (BINARY_SAVE) {
                    _socket->send(mv.to_binary(), sizeof(Motion_Vector_Global));
                } else {
                    std::ofstream myfile;
                    myfile.open("results/results_"+runnumber+"/monitor_v2x-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
                    myfile << mv.to_string();
                    myfile.close();
                }
            }
            // test.get
            // test.
            // if(test)
            // newdata.objDataProvider=&test->getVehicleData();
            // myfile<<test->getVehicleData();
            // myfile<<"test "<<newdata.objDataProvider->station_id()<<"\n";

            // myfile.close();

            
        }
        
        // newdata.objDataProvider=objDataProvider;

        
	}
}
