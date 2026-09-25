#include "EgoMotionController.h"
#include "artery/traci/VehicleController.h"
#include <boost/units/cmath.hpp>
#include "motion_vector.h"
#include <vanetza/btp/data_request.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>
#include <iostream>
#include <fstream>
#include <math.h> 
#include <boost/math/constants/constants.hpp>
#include <boost/units/cmath.hpp>

#include <curl/curl.h>
#include <stdio.h>
#include <string.h>
#include <exception>
#include <iterator>

#include <sys/types.h>
#include <sys/select.h>
#include <errno.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <string.h>

#define BOOST_GEOMETRY_DISABLE_DEPRECATED_03_WARNING

using namespace omnetpp;
using namespace vanetza;

Define_Module(EgoMotionController)

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}

// std::string get_data(std::string json){
//     std::string get_url = "https://iot.ufsc.br/api/get.php";
//     CURL *curl;
//     CURLcode res;
    
//     /* In windows, this will init the winsock stuff */
//     curl_global_init(CURL_GLOBAL_ALL);
    
//     /* get a curl handle */
//     curl = curl_easy_init();
//     std::string s;
//     if(curl) {
//         struct curl_slist *slist1 = NULL;
//         slist1 = curl_slist_append(slist1, "Content-Type: application/json");
//         slist1 = curl_slist_append(slist1, "Accept: application/json");
        
//         /* set custom headers */
//         curl_easy_setopt(curl, CURLOPT_HTTPHEADER, slist1);
        
//         curl_easy_setopt(curl, CURLOPT_URL, "https://iot.ufsc.br/api/get.php");
        
//         /* pass in a pointer to the data - libcurl does not copy */
//         curl_easy_setopt(curl, CURLOPT_POSTFIELDS, "{\"series\":{\"version\":\"1.2\",\"unit\":3298179364,\"r\":9999999,\"t0\":1672531200000000,\"tf\":1672531230000000,\"dev\":0,\"signature\":7777},\"credentials\":{\"domain\":\"tutorial\",\"username\":\"tutorial\",\"password\":\"tuto20182\"}}");

//         curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writefunc);
//         curl_easy_setopt(curl, CURLOPT_WRITEDATA, &s);
    
//         /* Perform the request, res will get the return code */
//         res = curl_easy_perform(curl);
//         if (EgoMotionController::HYSTERICALLY_DEBUG) {
//             std::ofstream myfile;
//             myfile.open("results/egomotion.out",  std::ios_base::app);
//             myfile << s;
//             myfile.close();
//         }
        
//         /* Check for errors */
//         if(res != CURLE_OK)
//         fprintf(stderr, "curl_easy_perform() failed: %s\n",
//                 curl_easy_strerror(res));
    
//         /* always cleanup */
//         curl_easy_cleanup(curl);
//     }
//     // curl_global_cleanup();

//     return s;

// }

void EgoMotionController::initialize()
{
    ItsG5BaseService::initialize();
    _mVehicleController = &getFacilities().get_const<traci::VehicleController>();

    controlinput = "/tmp/controlinput";
    controloutput = "/tmp/controloutput";

    libsumo::TraCIRoadPosition road;

    auto& m_vec = _mVehicleController->getTraCI()->vehicle;
    auto& m_sim = _mVehicleController->getTraCI()->simulation;
    _mVehicleDataProvider = &getFacilities().get_const<artery::VehicleDataProvider>();

    config_done=false;
    report_mv=false;

    std::vector<std::string> route;

    std::ofstream myfile;
    myfile.open("results/egomotion.out",  std::ios_base::app);

    mkfifo(controloutput.c_str(), 0666);
    mkfifo(controlinput.c_str(), 0666);

    controloutputid = open(controloutput.c_str(), O_WRONLY ); //open(myfifo, O_WRONLY | O_NONBLOCK);
    if (controloutputid == -1) {
        perror("open");
        return;
    }

    write(controloutputid, "Connect", sizeof("Connect"));
    close(controloutputid);

    
    controlinputid = open(controlinput.c_str(), O_RDONLY );
    char buffer[200];
    int test = read(controlinputid, &buffer[0], 200);
    while(test!=0) test = read(controlinputid, &buffer[0], 200);
    close(controlinputid);
    // myfile<<"Received from pipe: "<<buffer<<"\n";
    memset(buffer, 0, 200);

    myfile.close();

    config_done = this->setup_control();

    _route_is_set=true;

    consumer_index=0;
    
}

EgoMotionController::~EgoMotionController(){
    write_pipe("exitexitexit");
}

void EgoMotionController::trigger()
{
    Enter_Method("EgoMotionController trigger");

    std::ofstream myfile;
    if (DEBUG)
        myfile.open("results/egomotion.out",  std::ios_base::app);

    if(!config_done){
        config_done = this->setup_control();
    }

    if(!_route_is_set){
        _route_is_set = true;
        auto& m_vec = _mVehicleController->getTraCI()->vehicle;
        m_vec.setRoute(_mVehicleController->getVehicleId(), _route);
    }

    if(report_mv && config_done){
        Motion_Vector_Global s;
        s._id = _mVehicleDataProvider->station_id();
        auto microdegree = vanetza::units::degree * boost::units::si::micro;
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

        write_pipe("mv");
        std::string test = read_pipe();
        write_pipe(s.to_string());
    }

    if(speed_route_set){
        consumer_index++;
        unsigned long long t0 = _speed_data["series"][consumer_index-1]["timestamp"].template get<unsigned long long>();
        if (DEBUG)
            myfile << "\n\ncurrent index:" << consumer_index << ", sd size: " << _speed_data["series"].size() << "\ncurrent index time (sd): " << t0;
        unsigned long long t1 = _speed_data["series"][consumer_index]["timestamp"].template get<unsigned long long>();
        if (DEBUG)
            myfile << ", next index time (sd): " << t1;
        double ts_diff = (t1-t0)*FROM_MICRO_TO_SECONDS;
        if (DEBUG)
            myfile << ", diff time: " << ts_diff;

        float speed = _speed_data["series"][consumer_index]["value"].template get<float>();
        auto& m_vec = _mVehicleController->getTraCI()->vehicle;
        m_vec.slowDown(_mVehicleController->getVehicleId(), speed, ts_diff);

        if (DEBUG) {
            float previous_speed = _speed_data["series"][consumer_index-1]["value"].template get<float>();
            if (previous_speed < speed)
                myfile << "\nspeed up from " << previous_speed << "m/s to " << speed << "m/s, with constant acceleration over " << ts_diff << " seconds.";
            else
                myfile << "\nslow down from " << previous_speed << "m/s to " << speed << "m/s, with constant acceleration over " << ts_diff << " seconds.";
        }

    }

    if (DEBUG) {  
        auto& m_sim = _mVehicleController->getTraCI()->simulation;
        auto& m_vec = _mVehicleController->getTraCI()->vehicle;  
        myfile << "\nVehnicles current position:\n\tLane:" << m_vec.getLaneID(_mVehicleController->getVehicleId())
        <<" \n\tRoad:" << m_vec.getRoadID(_mVehicleController->getVehicleId())
        <<" \n\tPos:" << _mVehicleController->getPosition().x.value() << "," << _mVehicleController->getPosition().y.value() 
        << "\n\tGeoPos:" << _mVehicleController->getGeoPosition().latitude.value() << ","
        << _mVehicleController->getGeoPosition().longitude.value() 
        << "\n\tRoad from pos:" << m_sim.convertRoad(_mVehicleController->getPosition().x.value(), _mVehicleController->getPosition().y.value(), false).edgeID;
    }
    
    if (DEBUG)
        myfile.close();
}

std::vector<std::string> EgoMotionController::gen_route(json data){

    int data_size = data["series"].size();

    // auto& m_vec = _mVehicleController->getTraCI()->vehicle;
    // auto& m_sim = _mVehicleController->getTraCI()->simulation;
    // auto& m_lane = _mVehicleController->getTraCI()->lane;
    std::ofstream myfile;
    if (DEBUG) {
        myfile.open("results/egomotion.out",  std::ios_base::app);
        myfile << "\ngen_route()\n";
    }
    
    std::vector<std::string> route;
    std::string last = "";
    double latitude;
    double longitude;
    libsumo::TraCIRoadPosition road;

    auto& m_sim = _mVehicleController->getTraCI()->simulation;

    for(int i = 0; i < data_size; i++) {
        try{
            longitude = data["series"][i]["y"].template get<int>()/INTEGER_FLOAT_PRECISION;
            latitude  = data["series"][i]["x"].template get<int>()/INTEGER_FLOAT_PRECISION;

            if (HYSTERICALLY_DEBUG)
                myfile << "search for lane at longitude:" << longitude << ", latitude:" << latitude << "\n";
            
            auto pos = m_sim.convertGeo(longitude, latitude, true);
            road = m_sim.convertRoad(pos.x, pos.y, false);

            if (HYSTERICALLY_DEBUG)
                myfile << road.getString() << "\n";
        }
        catch(libsumo::TraCIException e){
        }

        if(road.edgeID.compare(last) != 0) {
            if (road.edgeID != "" && road.edgeID.find(":") == std::string::npos &&
                road.edgeID.find("_") == std::string::npos) {
            route.push_back(road.edgeID);
            if (DEBUG)
                myfile << road.edgeID << "/";
            last = road.edgeID;
            }
        }
    }

    if (DEBUG)
        myfile.close();

    return route;
}

void EgoMotionController::write_pipe(std::string data){
    controloutputid = open(controloutput.c_str(), O_WRONLY ); //open(myfifo, O_WRONLY | O_NONBLOCK);
    if (controloutputid == -1) {
        perror("open");
        return;
    } 
    write(controloutputid, data.c_str(), data.size());
    close(controloutputid);
}

std::string EgoMotionController::read_pipe(){

    controlinputid = open(controlinput.c_str(), O_RDONLY );
    char buffer[800];
    memset(buffer, 0, 800);
    int test = read(controlinputid, &buffer[0], 800);
    while(test!=0) test = read(controlinputid, &buffer[0], 800);
    close(controlinputid);

    return std::string(buffer);
}

bool EgoMotionController::setup_control(){

    std::ofstream myfile;
    myfile.open("results/egomotion.out",  std::ios_base::app);
    auto& m_vec = _mVehicleController->getTraCI()->vehicle;
    auto& m_sim = _mVehicleController->getTraCI()->simulation;

    myfile<<"start setup\n";

    bool quit=false;

    write_pipe("Connect");

    std::string inputstr = read_pipe();
    
    myfile<<"Received from pipe: "<<inputstr<<"\n";

    int command = atoi(inputstr.c_str());

    switch (command)
    {
        case END_CONFIG:{
            quit=true;
            break;
        }
        case SET_POS :{
            myfile<<"Setting position\n";
            write_pipe("Connect");
            inputstr = read_pipe();
            myfile<<"Received from pipe: "<<inputstr<<"\n";
            float xpos = std::stof(inputstr);
            write_pipe("Connect");
            inputstr = read_pipe();
            myfile<<"Received from pipe: "<<inputstr<<"\n";
            float ypos = std::stof(inputstr);
            libsumo::TraCIRoadPosition road;
            road = m_sim.convertRoad(xpos, ypos, false);
            m_vec.moveToXY(_mVehicleController->getVehicleId(), road.edgeID, -1, xpos, ypos, 0.0, 0);

            break;
        }

        case SET_TARGET :{
            myfile<<"Setting target\n";
            write_pipe("Connect");
            inputstr = read_pipe();
            myfile<<"Received from pipe: "<<inputstr<<"\n";
            float xpos = std::stof(inputstr);
            write_pipe("Connect");
            inputstr = read_pipe();
            myfile<<"Received from pipe: "<<inputstr<<"\n";
            float ypos = std::stof(inputstr);
            libsumo::TraCIRoadPosition road;
            road = m_sim.convertRoad(xpos, ypos, false);
            myfile<<"Road: "<<road.edgeID<<"\n";
            m_vec.changeTarget(_mVehicleController->getVehicleId(), road.edgeID);

            break;
        }

        case SET_REPORT_MV :{
            myfile<<"Setting target\n";
            report_mv=true;

            break;
        }

        case SET_ROUTE_SPEED:
            myfile<<"Setting route with speed\n";
            speed_route_set=true;

        case SET_ROUTE:{
            myfile<<"Setting route\n";
            write_pipe("Connect");
            inputstr = read_pipe();

            int positions = atoi(inputstr.c_str());

            std::string data="{\"series\":[";

            for(int i=0; i<positions; i++){
                if(i!=0) data.append(", ");
                write_pipe("Connect");
                inputstr = read_pipe();
                data.append(inputstr);
                
                // myfile<<inputstr<<"\n";
            }
            data.append("]}");

            _speed_data = json::parse(data);
            std::vector<std::string> routestr = gen_route(_speed_data);
            _route_is_set=false;

            break;
        }
        
        default:{
            break;
        }
    }


    myfile.close();

    return quit;
}


