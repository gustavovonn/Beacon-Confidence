#include "artery/application/Middleware.h"
#include "artery/networking/GeoNetIndication.h"
#include "artery/networking/GeoNetPacket.h"
#include "artery/networking/IDccEntity.h"
#include "artery/networking/PositionFixObject.h"
#include "artery/networking/Router.h"
#include "artery/networking/Runtime.h"
#include "artery/networking/SecurityEntity.h"
#include "artery/nic/RadioDriverBase.h"
#include "artery/nic/RadioDriverProperties.h"
#include "artery/utility/InitStages.h"
#include "artery/utility/PointerCheck.h"
#include <boost/units/cmath.hpp>
#include <boost/units/io.hpp>
#include <inet/common/ModuleAccess.h>
#include <vanetza/btp/header.hpp>
#include <vanetza/btp/header_conversion.hpp>
#include <vanetza/geonet/data_confirm.hpp>
#include <vanetza/net/cohesive_packet.hpp>
#include <vanetza/common/byte_buffer.hpp>
#include <vanetza/asn1/cam.hpp>
#include <fstream>



namespace vanetza {
namespace geonet {

static inline std::ostream& operator<<(std::ostream& os, const vanetza::geonet::LongPositionVector& epv)
{
    using namespace boost::units;
    os << "\n"
        << "latitude: \t" << abs(epv.position().latitude) << (epv.latitude.value() < 0 ? " S" : " N") << "\n"
        << "longitude: \t" << abs(epv.position().longitude) << (epv.longitude.value() < 0 ? " W" : " E") << "\n"
        << "heading: \t" << vanetza::units::GeoAngle { epv.heading };
    return os;
}

} // namespace geonet
} // namespace vanetza


namespace artery
{

Define_Module(Router)

static const omnetpp::simsignal_t scPositionFixSignal = omnetpp::cComponent::registerSignal("PositionFix");
static const omnetpp::simsignal_t scLinkReceptionSignal = omnetpp::cComponent::registerSignal("LinkReception");

int Router::numInitStages() const
{
    return InitStages::Total;
}

void Router::initialize(int stage)
{
    if (stage == InitStages::Prepare) {
        getParentModule()->subscribe(scPositionFixSignal, this);
        mMiddleware = inet::getModuleFromPar<Middleware>(par("middlewareModule"), this);
        mRadioDriver = inet::getModuleFromPar<RadioDriverBase>(par("radioDriverModule"), this);
        mRadioDriverDataIn = gate("radioDriverData");
        mRadioDriverPropertiesIn = gate("radioDriverProperties");
        mSecurityEntity = inet::findModuleFromPar<SecurityEntity>(par("securityModule"), this, false);
    } else if (stage == InitStages::Self) {
        // initialize MIB (will check for existence of security entity)
        initializeManagementInformationBase(mMIB);

        // basic router setup
        auto runtime = inet::getModuleFromPar<Runtime>(par("runtimeModule"), this);
        mRouter.reset(new vanetza::geonet::Router(*runtime, mMIB));
        vanetza::MacAddress init_mac = vanetza::create_mac_address(getId());
        mRouter->set_address(generateAddress(init_mac));

        // register security entity if available
        if (mSecurityEntity) {
            mRouter->set_security_entity(mSecurityEntity);
        }

        // bind router to DCC entity
        auto dccEntity = inet::findModuleFromPar<IDccEntity>(par("dccModule"), this);
        mRouter->set_access_interface(notNullPtr(dccEntity->getRequestInterface()));
        mRouter->set_dcc_field_generator(dccEntity->getGeonetFieldGenerator()); // nullptr is okay

        // pass BTP-B messages to transport layer dispatcher in network interface
        using vanetza::geonet::UpperProtocol;
        mNetworkInterface = std::make_shared<NetworkInterface>(*this, *dccEntity, mMiddleware->getTransportDispatcher());
        mRouter->set_transport_handler(UpperProtocol::BTP_B, &mNetworkInterface->getTransportHandler());

        // finally, register new network interface at middleware
        mMiddleware->registerNetworkInterface(mNetworkInterface);

        omnetpp::createWatch("EPV", mRouter->get_local_position_vector());
    }
}

void Router::finish()
{
    mRouter.reset();
}

void Router::receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t signal, omnetpp::cObject* obj, omnetpp::cObject*)
{
    if (signal == scPositionFixSignal) {
        auto fix = dynamic_cast<PositionFixObject*>(obj);
        if (fix && mRouter) {
            mRouter->update_position(*fix);
        }
    }
}

void Router::handleMessage(omnetpp::cMessage* msg)

{

    if (msg->getArrivalGate() == mRadioDriverDataIn) {
        auto* packet = omnetpp::check_and_cast<GeoNetPacket*>(msg);
        auto* indication = omnetpp::check_and_cast<GeoNetIndication*>(packet->getControlInfo());
        emit(scLinkReceptionSignal, packet);



        try {
            // Tenta indicar o payload para o router; se houver exceção, loga e descarta
            mRouter->indicate(std::move(*packet).extractPayload(), indication->source, indication->destination);
        } catch (const std::exception& e) {
            EV_WARN << "Router: Excecao ao processar payload na mRouter->indicate(): " << e.what() << "\n";
            EV_WARN << "Packet name: " << packet->getName()
                   << " | Packet byte length: " << packet->getByteLength()
                   << " | Indication arrival gate: " << msg->getArrivalGate()->getName()
                   << "\n";
            // Não relançamos a exceção — apenas descartamos o payload problemático.
        }
    } else if (msg->getArrivalGate() == mRadioDriverPropertiesIn) {
        auto* properties = omnetpp::check_and_cast<RadioDriverProperties*>(msg);
        auto addr = generateAddress(properties->LinkLayerAddress);
        mRouter->set_address(addr);
        Identity identity;
        identity.geonet.insert({mNetworkInterface, addr});
        emit(Identity::changeSignal, Identity::ChangeGeoNetAddress, &identity);
        mNetworkInterface->channel = properties->ServingChannel;
    } else {
        error("Do not know how to handle received message");
    }
    

    delete msg;
}


void Router::initializeManagementInformationBase(vanetza::geonet::ManagementInformationBase& mib)
{
    mib.itsGnDefaultTrafficClass.tc_id(3); // send BEACONs with DP3
    mib.itsGnIsMobile = par("isMobile");
    mib.itsGnSecurity = (mSecurityEntity != nullptr);
    mib.vanetzaDeferInitialBeacon = par("deferInitialBeacon");
    mib.vanetzaDisableBeaconing = par("disableBeaconing");
}

void Router::request(const vanetza::btp::DataRequestB& request, std::unique_ptr<vanetza::DownPacket> packet)
{ 
    
    if (request.destination_port.host() == 2001) {

    std::cout << "[SECURITY] Intercepted CAM request for port 2001. Original payload size: " << packet->layer(vanetza::OsiLayer::Application).size() << " bytes." << std::endl;

    CaObject* obj = reinterpret_cast <CaObject*>(packet->layer(vanetza::OsiLayer::Application).ptr());


    vanetza::asn1::Cam cam = obj->asn1();

    ItsPduHeader_t& header = (*cam).header;

    long stationId = header.stationID;



    std::cout << "[TEST] stationID=" << stationId << " messageID=" << header.messageID << std::endl;


    std::cout << "[TEST] CAM fields: " << header.stationID << ", " << header.messageID << std::endl;
    std::ofstream logFile(std::to_string(stationId) + "cam_log.txt", std::ios::out | std::ios::app);
    logFile << omnetpp::simTime()
            << " "
            << header.messageID
            << std::endl;
    logFile.close();


    //criar/abrir um arquivo para escrita do  messageID e simulation time(momento de envio do CAM)

      /* // 1. Extrair o payload serializado total
        std::vector<uint8_t> total_serialized;
        auto app_layer = packet->layer(vanetza::OsiLayer::Application);
        total_serialized.insert(total_serialized.end(), app_layer.begin(), app_layer.end());

        // 2. Acessar o objeto para extrair o stationID (para a chave)
        CaObject* obj = reinterpret_cast<CaObject*>(packet->layer(vanetza::OsiLayer::Application).ptr());
        vanetza::asn1::Cam original_cam = obj->asn1();
        long stationId = (*original_cam).header.stationID;

        // 3. Calcular o tamanho do Header para isolar o CoopAwareness
        std::vector<uint8_t> header_measure(256);
        asn_enc_rval_t enc_h = asn_encode_to_buffer(nullptr, ATS_UNALIGNED_BASIC_PER, 
                                                   &asn_DEF_ItsPduHeader, 
                                                   &((*original_cam).header), 
                                                   header_measure.data(), header_measure.size());
        size_t header_size = enc_h.encoded;

        // 4. Isolar e Criptografar apenas o CoopAwareness
        std::string ca_raw(reinterpret_cast<char*>(total_serialized.data() + header_size), 
                           total_serialized.size() - header_size);
        
        std::string groupKeyBin = get_group_key_bin(stationId);
        std::string encrypted_ca = encryptMessage(groupKeyBin.substr(0, 16), ca_raw);

        // 5. Calcular HMAC-SHA256 (32 bytes) sobre o payload cifrado
        std::string hmac_bin = calculateHMACSHA256_Bin(groupKeyBin, encrypted_ca); // Versão que retorna 32 bytes binários

        // 6. Montagem manual do NOVO Pacote (Header + Encrypted CA + HMAC)
        // O tamanho total será: tamanho do header + tamanho do cifrado + 32
        std::vector<uint8_t> secure_buffer;
        secure_buffer.reserve(header_size + encrypted_ca.size() + 32);

        // Adiciona Header
        secure_buffer.insert(secure_buffer.end(), header_measure.begin(), header_measure.begin() + header_size);
        // Adiciona Payload Cifrado
        secure_buffer.insert(secure_buffer.end(), encrypted_ca.begin(), encrypted_ca.end());
        // Adiciona HMAC Fixo (32 bytes)
        secure_buffer.insert(secure_buffer.end(), hmac_bin.begin(), hmac_bin.end());

        // 7. Substituir no DownPacket
        std::unique_ptr<vanetza::DownPacket> secure_packet { new vanetza::DownPacket() };
        vanetza::ByteBuffer b_buffer(secure_buffer.begin(), secure_buffer.end());
        secure_packet->layer(vanetza::OsiLayer::Application) = std::move(b_buffer);
        packet = std::move(secure_packet);

        std::cout << "[SECURITY] Sent Secure CAM: Header=" << header_size 
                  << " Payload=" << encrypted_ca.size() << " HMAC=32" << std::endl; */
    }

  

    ASSERT(mRouter);
    Enter_Method("request");
 
    using namespace vanetza;
    btp::HeaderB btp_header;
    btp_header.destination_port = request.destination_port;
    btp_header.destination_port_info = request.destination_port_info;
    packet->layer(OsiLayer::Transport) = btp_header;

    geonet::DataConfirm confirm;
    if (request.gn.transport_type == geonet::TransportType::SHB) {
        geonet::ShbDataRequest shb(mMIB);
        copy_request_parameters(request, shb);
        confirm = mRouter->request(shb, std::move(packet));
    } else if (request.gn.transport_type == geonet::TransportType::GBC) {
        geonet::GbcDataRequest gbc(mMIB);
        copy_request_parameters(request, gbc);
        confirm = mRouter->request(gbc, std::move(packet));
    } else {
        error("Unknown or unimplemented transport type");
    }

    if (confirm.rejected()) {
        error("GN-Data.request rejected");
    }
}

const vanetza::geonet::LocationTable& Router::getLocationTable() const
{
    if (!mRouter) {
        error("Router::getLocationTable called before initialization");
    }
    return mRouter->get_location_table();
}

const vanetza::geonet::LongPositionVector& Router::getEgoPositionVector() const
{
    if (!mRouter) {
        error("Router::getEgoPositionVector called before initialization");
    }
    return mRouter->get_local_position_vector();
}

vanetza::geonet::Address Router::getAddress() const
{
    if (!mRouter) {
        error("Router::getAddress called before initialization");
    }
    return mRouter->get_local_position_vector().gn_addr;
}

vanetza::geonet::Address Router::generateAddress(const vanetza::MacAddress& mac)
{
    vanetza::geonet::Address gnAddr;
    gnAddr.is_manually_configured(true);
    // VehicleMiddleware determines station type during initialisation
    gnAddr.station_type(mMiddleware->getStationType());
    gnAddr.country_code(0);
    gnAddr.mid(mac);
    return gnAddr;
}

} // namespace artery
