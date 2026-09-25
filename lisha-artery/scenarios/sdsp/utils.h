#ifndef UTILS_H_
#define UTILS_H_

#include "artery/application/ItsG5Service.h"
#include "artery/application/VehicleDataProvider.h"
#include "sdsp_msgs/ProtocolMessages_m.h"
#include <boost/units/quantity.hpp>
#include <vanetza/btp/data_request.hpp>
#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/profile.hpp>
#include <vanetza/geonet/interface.hpp>
#include <string>
#include <vector>
#include <vanetza/geonet/data_request.hpp>
#include <vanetza/geonet/areas.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/length.hpp>
#include <omnetpp.h>
#include <chrono>    
#include <random>
#include <sstream>
#include <iomanip>


// Constantes
#define M_PI 3.14159265358979323846
#define MICRODEGREE vanetza::units::degree * boost::units::si::micro


typedef artery::VehicleDataProvider VehicleDataProvider;
class BaseMessage;

void visitAuthPacketSender(BaseMessage* packet, const VehicleDataProvider* vehicleDataProvider = nullptr);

template<typename T>
vanetza::btp::DataRequestB createDataRequestB(const unsigned int port_num) {
    vanetza::btp::DataRequestB req;
    req.destination_port = vanetza::host_cast<typename T::port_type>(port_num);
    req.gn.transport_type = vanetza::geonet::TransportType::SHB;
    req.gn.traffic_class.tc_id(static_cast<unsigned>(vanetza::dcc::Profile::DP1));
    req.gn.communication_profile = vanetza::geonet::CommunicationProfile::ITS_G5;
    return req;
}


template<typename T>
inline vanetza::btp::DataRequestB createDataRequestWithAid(unsigned int itsAid) {
    vanetza::btp::DataRequestB req;
    req.destination_port = vanetza::host_cast<typename T::port_type>(itsAid);
    req.gn.transport_type = vanetza::geonet::TransportType::GBC; // GeoBroadcast
    req.gn.traffic_class.tc_id(static_cast<unsigned>(vanetza::dcc::Profile::DP1));
    req.gn.communication_profile = vanetza::geonet::CommunicationProfile::ITS_G5;
    return req;
}








template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u) {
    boost::units::quantity<U> v { q };
    return std::lround(v.value());
}


std::string bytesToHex(const uint8_t* bytes, size_t len);
std::vector<uint8_t> hexToBytes(const std::string& hex);
std::string sha256(const std::string& data);

bool openandWritekey(const std::string& key, const std::string& filename);
std::string readKey(const std::string& vehicleId);
std::string generateRandomKey();
void generateGroupKey(uint8_t* key, size_t len);

std::string encryptMessage(const std::string& key, const std::string& plaintext);
std::string decryptMessage(const std::string& key, const std::string& encryptedMessage);
std::string calculateOTP(const std::string& key, const std::string& data);


bool verifyOTP(const std::string& otp, const std::string& key, const std::string& data);

#endif // UTILS_H_
