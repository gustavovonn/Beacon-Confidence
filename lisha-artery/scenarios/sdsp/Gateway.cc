#include "Gateway.h"
#include "artery/application/LocalDynamicMap.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/tinycrypt/lib/include/tinycrypt/ecc_dh.h"
#include "artery/tinycrypt/lib/include/tinycrypt/ecc.h"
#include "artery/tinycrypt/lib/include/tinycrypt/hmac.h"
#include "artery/tinycrypt/lib/include/tinycrypt/sha256.h"
#include "artery/utility/Geometry.h"
#include "artery/utility/Identity.h"
#include "sdsp_msgs/ProtocolMessages_m.h"
#include "messageTypes.h"
#include "utils.h"
#include <omnetpp.h>
#include <string>
#include <sstream>
#include <iomanip>
#include <map>
#include <ctime>
#include <fcntl.h> // Para open()
#include <unistd.h> // Para read() e close()
#include <vanetza/btp/data_request.hpp>
#include <vanetza/geonet/interface.hpp>
#include <vanetza/geonet/traffic_class.hpp>

using namespace omnetpp;
using namespace artery;

Define_Module(Gateway);

// Função para gerar números aleatórios seguros usando /dev/urandom
int my_secure_rng_function(uint8_t *dest, unsigned int size) {
    int fd = open("/dev/urandom", O_RDONLY);
    if (fd == -1) {
        std::cerr << "ERROR: Failed to open /dev/urandom for RNG." << std::endl;
        return 0; // Falha ao abrir 
    }
    ssize_t result = read(fd, dest, size);
    close(fd);
    if (result != size) {
        std::cerr << "ERROR: Failed to read enough random bytes from /dev/urandom." << std::endl;
        return 0; // Falha ao ler bytes suficientes
    }
    return 1; //deu certo
}

void Gateway::initialize() {
    ItsG5Service::initialize();

    // Configurar o RNG seguro
    uECC_set_rng(&my_secure_rng_function);

    // Verificar se o RNG foi configurado corretamente
    if (uECC_get_rng() == nullptr) {
        throw cRuntimeError("RNG not configured. Cannot proceed.");
    }

    // Geração das chaves ECDH
    if (!generateECDHKeys()) {
        throw cRuntimeError("Failed to generate ECDH keys. Cannot proceed.");
    }

    // Inicialização da chave de grupo (exemplo)
    std::fill(std::begin(groupKey), std::end(groupKey), 0xAA); // Chave de grupo fixa para teste



    // //std::cout << "Gateway initialized successfully." << std::endl;
}

bool Gateway::generateECDHKeys() {
    // Verificar se a curva secp256r1 está disponível
    uECC_Curve curve = uECC_secp256r1();
    if (curve == nullptr) {
        std::cerr << "ERROR: secp256r1 curve not available." << std::endl;
        return false;
    }

    // Gerar chaves ECDH
    if (uECC_make_key(gatewayPublicKey, gatewayPrivateKey, curve)) {
        //std::cout << "ECDH keys generated successfully for Gateway." << std::endl;

        // Log das chaves geradas (apenas para depuração)
        //std::cout << "Public Key: ";
        for (int i = 0; i < NUM_ECC_BYTES * 2; i++) { 
            //std::cout << std::hex << std::setw(2) << (int)gatewayPublicKey[i];
        }
        //std::cout << std::endl;

        //std::cout << "Private Key: ";
        for (int i = 0; i < NUM_ECC_BYTES; i++) {
            //std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)gatewayPrivateKey[i];
        }
        //std::cout << std::endl;

        return true;
    } else {
        std::cerr << "ERROR: Failed to generate ECDH keys for Gateway." << std::endl;
        return false;
    }
}

void Gateway::handleMessage(cMessage* msg) {
    if (auto secureMsg = dynamic_cast<SecureMessage*>(msg)) {
        processSecureMessage(secureMsg);
    } else {
        ItsG5Service::handleMessage(msg);
    }
}

void Gateway::indicate(const vanetza::btp::DataIndication& ind, cPacket* packet) {
    Enter_Method("Gateway indicate");

    auto msg = check_and_cast<const BaseMessage*>(packet);
    //std::cout << "Indication received, message type: " << msg->getType() << " from vehicle id: " << msg->getVehicleId() << std::endl;

    switch (msg->getType()) {
        case PTP_REQUEST: processPtpRequest(check_and_cast<const PtpRequest*>(msg)); break;
        case ECDH_REQUEST: processECDHRequest(check_and_cast<const ECDHRequest*>(msg)); break;
        case AUTH_REQUEST: processAuthRequest(check_and_cast<const AuthRequest*>(msg)); break;
        case ACTIVE_MESSAGE: processActiveMessage(check_and_cast<const ActiveMessage*>(msg)); break;
        case SECURE_MESSAGE: processSecureMessage(check_and_cast<const SecureMessage*>(msg)); break;
        default: std::cout << "Unknown message type received: " << msg->getType() << std::endl; break;
    }
    delete msg;
}

void Gateway::processPtpRequest(const PtpRequest* ptpRequest) {
    auto ptpResponse = new PtpResponse();
    ptpResponse->setT2(simTime().inUnit(SIMTIME_US)); // T2 = tempo de recebimento
    ptpResponse->setT3(simTime().inUnit(SIMTIME_US)); // T3 = tempo de envio
    ptpResponse->setT1(ptpRequest->getT1()); // manda T1 para o Device
    ptpResponse->setType(PTP_RESPONSE);
    ptpResponse->setVehicleId(ptpRequest->getVehicleId());
    ptpResponse->setByteLength(40);

    vanetza::btp::DataRequestB req = createDataRequestB<Gateway>(getPortNumber());
    req.gn.traffic_class = vanetza::geonet::TrafficClass(0); 
    req.gn.transport_type = vanetza::geonet::TransportType::SHB;

    request(req, ptpResponse);

    //std::cout << "Sent PtpResponse to " << ptpRequest->getVehicleId() << std::endl;
}


void Gateway::processECDHRequest(const ECDHRequest* ecdhRequest) {
    std::string devicePublicKeyHex = ecdhRequest->getYd();
    //std::cout << "Device public key (received as hex): " << devicePublicKeyHex << std::endl;

    //Converte a string hexadecimal recebida para um vetor de bytes de forma segura.
    // Isso evita o erro de buffer overflow que ocorria antes.
    std::vector<uint8_t> devicePublicKeyVec = hexToBytes(devicePublicKeyHex);
    
    //Verificação de segurança para garantir que a chave pública tenha o tamanho esperado (64 bytes).
    if (devicePublicKeyVec.size() != 64) {
        std::cerr << "ERROR: Received public key from device " << ecdhRequest->getVehicleId() 
                  << " has incorrect size. Expected 64, got " << devicePublicKeyVec.size() << "." << std::endl;
        return;
    }
    
    
    uint8_t sharedSecret[32];
    if (uECC_shared_secret(devicePublicKeyVec.data(), gatewayPrivateKey, sharedSecret, uECC_secp256r1())) {
        std::string deviceId = ecdhRequest->getVehicleId();
        
        
        std::array<uint8_t, 32> secretArray;
        std::copy(std::begin(sharedSecret), std::end(sharedSecret), secretArray.begin());
        
        sharedSecrets[deviceId] = secretArray;

        // Log para depuração
         //std::cout << "Shared Secret for " << deviceId << ": " << bytesToHex(sharedSecret, sizeof(sharedSecret)) << std::endl;
        // std::cout << "Shared secret calculated and stored for device: " << deviceId << std::endl;

    } else {
        std::cerr << "ERROR: Failed to calculate shared secret for device: " << ecdhRequest->getVehicleId() << std::endl;
        return; // Retorna para não continuar com dados inválidos.
    }

    // --- Envio da Resposta ---
    auto ecdhResponse = new ECDHResponse();
    ecdhResponse->setType(ECDH_RESPONSE);
    ecdhResponse->setVehicleId(ecdhRequest->getVehicleId());
    
    // CRÍTICO! Converte a chave pública binária do gateway para uma string hexadecimal antes de enviar.
    // Isso garante que a chave não seja corrompida ou truncada durante a transmissão pela rede.
    std::string gatewayPublicKeyHex = bytesToHex(gatewayPublicKey, sizeof(gatewayPublicKey));
    ecdhResponse->setYg(gatewayPublicKeyHex.c_str());
    
    // Log para depuração
     ////std::cout << "Gateway Public Key (Sent as Hex): " << gatewayPublicKeyHex << std::endl;

    ecdhResponse->setByteLength(150); // Tamanho de pacote mais realista para uma chave de 64 bytes em hexadecimal

    vanetza::btp::DataRequestB req = createDataRequestB<Gateway>(getPortNumber());
    req.gn.traffic_class = vanetza::geonet::TrafficClass(0); 
    req.gn.transport_type = vanetza::geonet::TransportType::SHB;

    request(req, ecdhResponse);

     //std::cout << "Sent ECDHResponse to " << ecdhRequest->getVehicleId() << std::endl;
}

void Gateway::processAuthRequest(const AuthRequest* authRequest) {
    std::string deviceId = authRequest->getVehicleId();
    std::string authId = "AUTHD" + deviceId;




    if (validAuthdList.find(authId) == validAuthdList.end()) {
        std::cerr << "Device " << deviceId << " is not authorized to authenticate." << std::endl;
        return;
    }

    // MOTIVO: Busca o segredo no map de forma segura.
    auto it = sharedSecrets.find(deviceId);
    if (it != sharedSecrets.end()) {
        // MOTIVO: Acessa o std::array e cria a chave para o OTP.
        const auto& secretArray = it->second;
        std::string key(reinterpret_cast<const char*>(secretArray.data()), secretArray.size());

        std::string auth = "AUTHD" + deviceId;
        std::string message = "MessageData";
        std::string timestamp = authRequest->getTimestamp();
        

         //std::cout << "Key gateway: " << auth << std::endl;
        // std::cout << "Message gateway: " << message << std::endl;
        // //std::cout << "Timestamp gateway: " << timestamp << std::endl;
        std::string fullTimestamp = std::to_string (simTime().inUnit(SIMTIME_US)/100000);
        std::string calculatedOtp = calculateOTP(key, auth + message + fullTimestamp);
        std::string truncatedOtp = calculatedOtp; ////.substr(0, 6);

        std::string fullTimestamp1 = std::to_string ((simTime().inUnit(SIMTIME_US)/100000)-1);
        std::string calculatedOtp1= calculateOTP(key, auth + message + fullTimestamp1);
        std::string truncatedOtp1 = calculatedOtp1; ////.substr(0, 6)

        std::string fullTimestamp2 = std::to_string ((simTime().inUnit(SIMTIME_US)/100000)+1);
        std::string calculatedOtp2 = calculateOTP(key, auth + message + fullTimestamp2);
        std::string truncatedOtp2= calculatedOtp2; ////.substr(0, 6)

    

        if (truncatedOtp == authRequest->getOTPd() || truncatedOtp1 == authRequest->getOTPd()|| truncatedOtp2 == authRequest->getOTPd() ) {
            auto grant = new AuthGranted();
            grant->setType(AUTH_GRANTED);
            grant->setVehicleId(deviceId.c_str());

            // MOTIVO: Converte a chave de grupo (KG) e a chave pública (CYg) para hexadecimal antes de enviar.
            grant->setKG(bytesToHex(groupKey, sizeof(groupKey)).c_str());
            grant->setCYg(bytesToHex(gatewayPublicKey, sizeof(gatewayPublicKey)).c_str());
            
            grant->setAuthg(auth.c_str());
            grant->setByteLength(250); // somae  aes
            

           vanetza::btp::DataRequestB req = createDataRequestB<Gateway>(getPortNumber());

           req.gn.traffic_class = vanetza::geonet::TrafficClass(0); 
           req.gn.transport_type = vanetza::geonet::TransportType::SHB;
            

            request(req, grant);
        

            // std::cout << "Sent AuthGranted to device: " << deviceId << std::endl;
        } else {
            std::cerr << "OTP mismatch. Authentication failed for device: " << deviceId << std::endl;
            std::cerr << "Received OTP: " << authRequest->getOTPd() << std::endl;
        }
    } else {
        std::cerr << "No shared secret found for device: " << deviceId << std::endl;
    }
}





void Gateway::processActiveMessage(const ActiveMessage* activeMessage) {
    // //std::cout << "Active message from vehicle " << activeMessage->getVehicleId() << std::endl;

    if (validateMessage(activeMessage)) {
        auto response = new ActiveMessage();
        response->setAuthh(activeMessage->getAuthh());
        response->setOTPpd(activeMessage->getOTPpd());
        response->setAuthde(activeMessage->getAuthde());
        response->setByteLength(40);

        vanetza::btp::DataRequestB req = createDataRequestB<Gateway>(getPortNumber());
        request(req, response);

        // std::cout << "Forwarded ActiveMessage to target vehicle: " << activeMessage->getAuthde() << std::endl;
    } else {
       //  //std::cout << "Message validation failed. Possible impostor attempt." << std::endl;
    }
}

void Gateway::processSecureMessage(const SecureMessage* secureMsg) {

    // //std::cout << "Gateway saw a SecureMessage, but will ignore it (as expected)." << std::endl;

}

bool Gateway::validateMessage(const SecureMessage* secureMsg) {
    // Extrair Authd, KG, payload, timestamp da SecureMessage
    //Gerar HMAC usando sharedSecret ou groupKey

//Comparar com MAC recebido
    return true;
}

bool Gateway::validateMessage(const ActiveMessage* message) {
    
    return true;
}