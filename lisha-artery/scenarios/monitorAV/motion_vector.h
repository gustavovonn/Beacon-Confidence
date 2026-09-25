#ifndef MOTION_VECTOR_H_
#define MOTION_VECTOR_H_

#include <iostream>
#include <fstream>
#include <string>
#include <math.h> 
#include <boost/math/constants/constants.hpp>
#include <boost/units/cmath.hpp>
#include <random>

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/networking/PositionProvider.h"
#include "artery/utility/Geometry.h"
#include "traci/sumo/utils/traci/TraCIAPI.h"
#include "json.hpp"
#include <curl/curl.h>

typedef unsigned int PERCENTAGE_0_100;
typedef unsigned long long Time;

class F32_SD {
public:
    F32_SD() = default;
public:
    std::string to_string() {
        return "{\"value\":" + std::to_string(_value) + ",\"uncertainty\":" + std::to_string(_uncertainty) + "}";
    }

public:
    float _value                    = 0;
    PERCENTAGE_0_100 _uncertainty   = 100;
};

class F64_SD {
public:
    F64_SD() = default;
public:
    std::string to_string() {
        return "{\"value\":" + std::to_string(_value) + ",\"uncertainty\":" + std::to_string(_uncertainty) + "}";
    }

public:
    double _value                    = 0;
    PERCENTAGE_0_100 _uncertainty   = 100;
};

class Geo_Point { // radians
public:
    Geo_Point() = default;

public:
    std::string to_string() {
        return "[" + _lat.to_string() + "," + _lon.to_string() + "," + _alt.to_string() + "]";
    }

public:
    F64_SD _lon;
    F64_SD _lat;
    F64_SD _alt;
};

class Point_2D { // meters
public:
    Point_2D() = default;

public:
    std::string to_string() {
        return "[" + _x.to_string() + "," + _y.to_string() + "]";
    }
public:
    F32_SD _x;
    F32_SD _y;
};

class Dimensions { // meters
public:
    Dimensions() = default;

public:
    std::string to_string() {
        return "[" + _width.to_string() + "," + _length.to_string() + "," + _height.to_string() + "]";
    }
public:
    F32_SD _width;
    F32_SD _length;
    F32_SD _height;
};

class Motion_Vector_Global {
public:
    Motion_Vector_Global() = default;
public:
    std::string to_string() {
        std::string ret = "{\"Motion_Vector_Global\":{";
        ret += "\"origin\":{\"id\":" + std::to_string(_id) +",\"idlocal\":"+ std::to_string(_idlocal) + ",\"timestamp\":" + std::to_string(_timestamp) + "}";
        ret += ",\"unit\":\"Digital,MV_Global\",\"class\":" + std::to_string(_class) + ",\"uncertainty\":" + std::to_string(_uncertainty) + ",\"length\":1" ;
        ret += ",\"loc\":" + _location.to_string();
        ret += ",\"speed\":" + _speed.to_string();
        ret += ",\"heading\":" + _heading.to_string();
        ret += ",\"yaw_rate\":" + _yaw_rate.to_string();
        ret += ",\"acceleration\":" + _acceleration.to_string();
        ret += ",\"dim\":" + _dimension.to_string();
        ret += "}}\n";
        return ret;
    }

    unsigned char * to_binary() {
        return reinterpret_cast<unsigned char *>(this);
    }

    Motion_Vector_Global copy_with_noise(float distance, const TraCIAPI::SimulationScope &_msim){
        Motion_Vector_Global s;

        const double mean = 0.0;
        const double stddev = 0.01 * (((int) distance) / 20);
        // std::cout<<"\nstart "<<stddev<<"\n";
        auto dist = std::bind(std::normal_distribution<double>{mean, stddev},
                            std::mt19937(std::random_device{}()));

        auto objpos = _msim.convertGeo(this->_location._lat._value/(M_PI/180.0)/10000000, this->_location._lon._value/(M_PI/180.0)/10000000, true);

        // std::cout<<"orig x:"<<objpos.x<<" y:"<<objpos.y<<"\n";
        objpos.x +=dist();
        objpos.y +=dist();
        // std::cout<<"alte x:"<<objpos.x<<" y:"<<objpos.y<<"\n";

        auto objpos2 = _msim.convertGeo(objpos.x, objpos.y, false);

        s._id = this->_id;
        s._idlocal = this->_idlocal;
        s._location._lat._value = objpos2.x;
        s._location._lat._uncertainty = 0; // TODO: from elipse to uncertainty
        s._location._lon._value = objpos2.y;
        s._location._lon._uncertainty = 0;
        s._location._alt._value = 0; // AltitudeValue_unavailable
        s._location._alt._uncertainty = 100;
        s._speed._value = this->_speed._value + dist();
        s._speed._uncertainty = 0;
        s._heading._value = this->_heading._value + dist();
        s._heading._uncertainty = 0;
        s._yaw_rate._value = this->_yaw_rate._value + dist();
        s._yaw_rate._uncertainty = 0;
        s._acceleration._value = this->_acceleration._value + dist();
        s._acceleration._uncertainty = 0;
        s._dimension._length._value = this->_dimension._length._value;
        s._dimension._length._uncertainty = 0;
        s._dimension._width._value = this->_dimension._width._value;
        s._dimension._width._uncertainty = 0;
        s._dimension._height._value = 0;
        s._dimension._height._value = 100;
        s._class = 5; // StationType_passengerCar
        s._uncertainty = 0;
        s._timestamp = this->_timestamp;

        return s;
    }

    Motion_Vector_Global copy_simple(){
        Motion_Vector_Global s;

        s._id = this->_id;
        s._idlocal = this->_idlocal;
        s._location._lat._value = this->_location._lat._value;
        s._location._lat._uncertainty = 0; // TODO: from elipse to uncertainty
        s._location._lon._value = this->_location._lon._value;
        s._location._lon._uncertainty = 0;
        s._location._alt._value = 0; // AltitudeValue_unavailable
        s._location._alt._uncertainty = 100;
        s._speed._value = this->_speed._value;
        s._speed._uncertainty = 0;
        s._heading._value = this->_heading._value;
        s._heading._uncertainty = 0;
        s._yaw_rate._value = this->_yaw_rate._value;
        s._yaw_rate._uncertainty = 0;
        s._acceleration._value = this->_acceleration._value;
        s._acceleration._uncertainty = 0;
        s._dimension._length._value = this->_dimension._length._value;
        s._dimension._length._uncertainty = 0;
        s._dimension._width._value = this->_dimension._width._value;
        s._dimension._width._uncertainty = 0;
        s._dimension._height._value = 0;
        s._dimension._height._value = 100;
        s._class = 5; // StationType_passengerCar
        s._uncertainty = 0;
        s._timestamp = this->_timestamp;

        return s;
    }

public:
    // Part of MV Unit
    unsigned short   _class; 
    PERCENTAGE_0_100 _uncertainty; // uncertainty inherent to unit

    // part of origin information
    unsigned long long _id;
    unsigned long long _idlocal=0;
    Time _timestamp;

    Geo_Point _location;    // 3*rad
    F32_SD _speed;          // m/s
    F32_SD _heading;        // rad
    F32_SD _yaw_rate;       // rad/s
    F32_SD _acceleration;   // m/s2
    Dimensions _dimension;  //3*m
};

class Motion_Vector_Local {
public:
    Motion_Vector_Local() = default;

public:
    std::string to_string() {
        std::string ret = "{\"Motion_Vector_Local\":{";
        ret += "\"origin\":{\"id\":" + std::to_string(_id) + ",\"timestamp\":" + std::to_string(_timestamp) + "}";
        ret += ",\"unit\":\"Digital,MV_Local\",\"class\":" + std::to_string(_class) + ",\"uncertainty\":" + std::to_string(_uncertainty) + ",\"length\":1" ;
        ret += ",\"loc\":" + _location.to_string();
        ret += ",\"speed\":" + _speed.to_string();
        ret += ",\"heading\":" + _heading.to_string();
        ret += ",\"yaw_rate\":" + _yaw_rate.to_string();
        ret += ",\"acceleration\":" + _acceleration.to_string();
        ret += ",\"dim\":" + _dimension.to_string();
        ret +=  "}}\n";
        return ret;
    }

    unsigned char * to_binary() {
        return reinterpret_cast<unsigned char *>(this);
    }

public:
    // Part of MV Unit
    unsigned short   _class; 
    PERCENTAGE_0_100 _uncertainty; // uncertainty inherent to unit

    // part of origin information
    unsigned long long _id;
    Time _timestamp;

    Point_2D _location;     // 3*rad
    F32_SD _speed;          // m/s
    F32_SD _heading;        // rad
    F32_SD _yaw_rate;       // rad/s
    F32_SD _acceleration;   // m/s2
    Dimensions _dimension;  //3*m

};

class AV_State : public Motion_Vector_Global {
public:
    AV_State() = default;

public:
    std::string to_string() {
        std::string ret = "{\"State\":{";
        ret += "\"origin\":{\"id\":" + std::to_string(_id) + ",\"timestamp\":" + std::to_string(_timestamp) + "}";
        ret += ",\"unit\":\"Digital,State\",\"class\":" + std::to_string(_class) + ",\"uncertainty\":" + std::to_string(_uncertainty) + ",\"length\":1\"" ;
        ret += ",\"loc\":" + _location.to_string();
        ret += ",\"speed\":" + _speed.to_string();
        ret += ",\"heading\":" + _heading.to_string();
        ret += ",\"yaw_rate\":" + _yaw_rate.to_string();
        ret += ",\"acceleration\":" + _acceleration.to_string();
        ret += ",\"dim\":" + _dimension.to_string();
        ret += ",\"orientation\":" + _orientation.to_string();
        ret += ",\"steering\":" + _steering.to_string();
        ret += ",\"parking\":" + std::to_string(_parking);
        ret += "}}\n";
        return ret;
    }

    unsigned char * to_binary() {
        return reinterpret_cast<unsigned char *>(this);
    }

public:
    F32_SD _orientation; // rad
    F32_SD _steering;    // rad
    bool _parking;       // switch
};

using json = nlohmann::json;

std::string static create_series(unsigned long unit, unsigned long t0, unsigned long tf, int dev, std::string sig){
    json cseries;
    cseries["series"] = { {"version", "1.2"}, {"unit", unit}, {"t0", t0}, {"t1", tf}, {"workflow", 0}, {"dev", dev}, {"signature", sig} };
    cseries["credentials"]= { {"domain", "tutorial"},  {"username", "tutorial"}, {"password", "tuto20182"} };

    return cseries.dump();
}

// void static httppost(std::string url, std::string content){
//     cpr::Response r = cpr::Post(cpr::Url{url},
//                       cpr::Body{content.c_str()},
//                       cpr::Header{{"Content-type", "application/json"}});


//     if(r.status_code !=200){
//         std::cout <<r.status_code << "\n";                  // 200
//         std::cout <<r.text << "\n";                        // JSON text string
//     }
//     else std::cout <<r.text << "\n";

//     // httplib::SSLClient cli("iot.lisha.ufsc.br", 443);
//     // cli.enable_server_certificate_verification(false);
//     // auto res = cli.Post("/api/get.php", content.c_str(), strlen(content.c_str()) , "text/plain");

//     // if (res) {
//     //     std::ofstream myfile;
//     //     myfile.open("results/test.out",  std::ios_base::app);
//     //     myfile << res->status << "\n";
//     //     myfile << res->get_header_value("Content-Type") << "\n";
//     //     myfile << res->body << "\n";
//     //     myfile.close();
//     // } else {
//     //     std::ofstream myfile;
//     //     myfile.open("results/test.out",  std::ios_base::app);
//     //     myfile << "error code: " << res.error() << "\n";;
//     //     myfile.close();
//     // }
// }

// void static httpput(std::string url, std::string content){
//     cpr::Response r = cpr::Post(cpr::Url{url},
//                       cpr::Body{content.c_str()},
//                       cpr::Header{{"Content-type", "application/json"}});


//     if(r.status_code !=200){
//         std::cout <<r.status_code << "\n";                  // 200
//         std::cout <<r.text << "\n";                        // JSON text string
//         std::cout <<r.reason << "\n";
//         // std::cout <<r.error << "\n";
//     }
//     else std::cout <<r.text << "\n";


// }

std::vector<std::string> static initSD(std::string url, unsigned long t0, unsigned long tf, std::string sig){

    unsigned long velocity_F32 = 0xC4963924;
    unsigned long rad_F32      = 0xC4B24924;
    unsigned long rad_per_sec_F32 = 0xC4B23924;
    unsigned long m_per_sec2_F32 = 0xC4962924;

    std::cout<< "t0: "<<t0<<" tf: "<<tf<<"\n";

    std::string speed = create_series(velocity_F32, t0, tf, 0, sig); //speed
    std::string heading = create_series(rad_F32, t0, tf, 7, sig); //heading
    std::string yawrate = create_series(rad_per_sec_F32, t0, tf, 3, sig); //yaw rate
    std::string accel = create_series(m_per_sec2_F32, t0, tf, 3, sig); //acceleration

    std::vector<std::string> returnlist;

    returnlist.push_back(speed+"\n");
    returnlist.push_back(heading+"\n");
    returnlist.push_back(yawrate+"\n");
    returnlist.push_back(accel+"\n");

    return returnlist;
}

std::string static create_sd(unsigned long unit, unsigned long ts, float value, unsigned long x, unsigned long y, unsigned long z, int dev, std::string sig){
    json cseries;
    cseries["smartdata"] = {{ {"version", "1.2"}, {"unit", unit}, {"timestamp", ts}, {"value", value}, {"error", 0},  {"confidence", 1}, {"x", x},  {"y", y},  {"z", z}, {"dev", dev}, {"signature", sig} }};
    cseries["credentials"]= { {"domain", "tutorial"},  {"username", "tutorial"}, {"password", "tuto20182"} };

    return cseries.dump();
}

std::vector<std::string>  static uploadSD(std::string url, Motion_Vector_Global mvg){
    unsigned long velocity_F32 = 0xC4963924;
    unsigned long rad_F32      = 0xC4B24924;
    unsigned long rad_per_sec_F32 = 0xC4B23924;
    unsigned long m_per_sec2_F32 = 0xC4962924;

    std::string speed = create_sd(velocity_F32, mvg._timestamp, mvg._speed._value, mvg._location._lat._value, mvg._location._lon._value, 0, 0, std::to_string(mvg._id)); //speed
    std::string heading = create_sd(rad_F32, mvg._timestamp, mvg._heading._value, mvg._location._lat._value, mvg._location._lon._value, 0, 7, std::to_string(mvg._id)); //heading
    std::string yawrate = create_sd(rad_per_sec_F32, mvg._timestamp, mvg._yaw_rate._value, mvg._location._lat._value, mvg._location._lon._value, 0, 3, std::to_string(mvg._id)); //yaw rate
    std::string accel = create_sd(m_per_sec2_F32, mvg._timestamp, mvg._acceleration._value, mvg._location._lat._value, mvg._location._lon._value, 0, 3, std::to_string(mvg._id)); //acceleration

    std::vector<std::string> returnlist;

    returnlist.push_back(speed+"\n");
    returnlist.push_back(heading+"\n");
    returnlist.push_back(yawrate+"\n");
    returnlist.push_back(accel+"\n");

    return returnlist;

}

#endif /* MOTION_VECTOR_H_ */