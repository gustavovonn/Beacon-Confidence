#ifndef GATEWAY_H_
#define GATEWAY_H_

#include "artery/application/ItsG5Service.h"
#include <omnetpp.h>
#include "messageTypes.h"
#include "artery/utility/Channel.h"
#include <omnetpp/simtime.h>
#include <set>  
#include <string>
#include <map>
#include "artery/tinycrypt/lib/include/tinycrypt/ecc_dh.h"
#include "artery/tinycrypt/lib/include/tinycrypt/ecc.h"
#include "artery/tinycrypt/lib/include/tinycrypt/hmac.h"
#include "artery/tinycrypt/lib/include/tinycrypt/sha256.h"
#include "artery/arduinolibs/libraries/Crypto/Poly1305.h"
#include "sdsp_msgs/ProtocolMessages_m.h"
#include "utils.h"
#include <vanetza/asn1/tstp.hpp>
#include <array>

using namespace artery;

class Gateway : public artery::ItsG5Service {
public:
    void handleMessage(omnetpp::cMessage* msg) override;
    void initialize() override;
    void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*) override;

    private:
    // ...
    uint8_t gatewayPublicKey[64];
    uint8_t gatewayPrivateKey[32]; 
    uint8_t groupKey[16];
    
    std::map<std::string, std::array<uint8_t, 32>> sharedSecrets;
    
    struct SecureSession {
        std::string vehicleId;
        uint8_t sharedKey[32];
    };

    // Lista de Authd válidos
    
    std::set<std::string> validAuthdList = {"AUTHD0.0", "AUTHD1.0"}; 

    // Funções para processar mensagens
    void processPtpRequest(const PtpRequest* ptpRequest);
    void processECDHRequest(const ECDHRequest* ecdhRequest);
    void processActivationRequest(const ActivationRequest* activationRequest);
    void processAuthRequest(const AuthRequest* authRequest);
    void processActiveMessage(const ActiveMessage* activeMessage);
    void processSecureMessage(const SecureMessage* secureMsg);

    // Funções para enviar mensagens
    void sendPtpResponse(const PtpRequest* ptpRequest);
    void sendECDHResponse(const ECDHRequest* ecdhRequest);
    void sendAuthGranted(const AuthRequest* authRequest);
    void sendSecureMessage(const SecureMessage* secureMsg, const std::string& Authd);

    // Funções para criptografar, descriptografar e validar mensagens
    bool validateMessage(const SecureMessage* secureMsg);
    bool validateMessage(const ActiveMessage* msg);


    std::string xorStrings(const std::string& str1, const std::string& str2);

    // Canais e tabelas de rede
    ChannelNumber mPrimaryChannel = channel::CCH;
    const NetworkInterfaceTable* mNetworkInterfaceTable = nullptr;
    const Timer* mTimer = nullptr;
    const Identity* mIdentity = nullptr;
    const GeoPosition* mGeoPosition = nullptr;
    LocalDynamicMap* mLocalDynamicMap = nullptr;

    // Métodos da classe
    bool generateECDHKeys(); 
    void generateRandomKey(uint8_t* key, size_t keyLength);
    void generateGroupKey();

    // Variáveis de tempo
    omnetpp::SimTime lastSyncTime = 0;
    omnetpp::SimTime syncTimeout = 1000;  // Timeout para sincronização de tempo
    omnetpp::SimTime lastPtpRequestTime = 0;
    const omnetpp::SimTime AUTH_TIMEOUT_S = 100000; // Timeout para a autorização

};

#endif /* GATEWAY_H_ */