#ifndef DEVICE_H_
#define DEVICE_H_

#include "artery/application/ItsG5Service.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/traci/VehicleController.h"
#include "sdsp_msgs/ProtocolMessages_m.h"
#include "artery/tinycrypt/lib/include/tinycrypt/ecc_dh.h"
#include "artery/tinycrypt/lib/include/tinycrypt/ecc.h"
#include "artery/tinycrypt/lib/include/tinycrypt/hmac.h"
#include "artery/tinycrypt/lib/include/tinycrypt/sha256.h"
#include "artery/arduinolibs/libraries/Crypto/Poly1305.h"
#include <vanetza/asn1/tstp.hpp>
#include "utils.h"
#include <array>
#include <string>
#include <omnetpp.h>
#include <set>  
#include <map>

using namespace artery;

class Device : public ItsG5Service {
public:
    static const unsigned int BASEMESSAGESIZE = (4+3+32+8+8);
    static const unsigned int PTP_REQUESTSIZE = (BASEMESSAGESIZE)+(8);
    static const unsigned int ECDH_REQUESTSIZE = (BASEMESSAGESIZE)+(64);
    static const unsigned int AUTH_REQUESTSIZE = (BASEMESSAGESIZE)+(64);
    void initialize() override;
    void indicate(const vanetza::btp::DataIndication&, omnetpp::cPacket*) override;
    void trigger() override;
    void finish() override;
    void generateECDHKeys(); 
    std::string otp;

private:
    omnetpp::SimTime adjustedTime; 

    uint8_t devicePublicKey[64];   // Chave pública ECDH (64 bytes)
    uint8_t devicePrivateKey[32];  // Chave privada ECDH (32 bytes)
    uint8_t sharedSecret[32];      // Segredo compartilhado ECDH
    uint8_t groupKey[16];          // Chave de grupo (16 bytes)
    
    // Flags de sincronização e autorização
    bool isAuthorized = false;
    bool authrequested =false;
    bool messageSent = false; 
    bool isTimeSynced = false;
    bool ecdhComplete = false;
    bool ecdhRequested = false;
    bool isAuthenticating;

    // Variáveis de tempo
    omnetpp::simtime_t authStartTime;
    omnetpp::simtime_t kgRequestStartTime;   // <-- ADICIONADA
    omnetpp::SimTime lastSyncTime = 0;
    omnetpp::SimTime syncTimeout = 1000;     // Timeout para sincronização de tempo
    omnetpp::SimTime lastPtpRequestTime = 0;
    const omnetpp::SimTime AUTH_TIMEOUT_S = 100000; // Timeout para a autorização

    // Sinais de medição de latência
    omnetpp::simsignal_t authLatencySignal;
    omnetpp::simsignal_t v2vLatencySignal;
    omnetpp::simsignal_t ecdhLatencySignal;
    omnetpp::simsignal_t otpLatencySignal;
    omnetpp::simsignal_t aesEncryptLatencySignal;
    omnetpp::simsignal_t aesDecryptLatencySignal;
    omnetpp::simsignal_t groupKeyLatencySignal;

    // Métodos para geração de chaves e envio de mensagens
    void sendPtpRequest();
    void sendECDHRequest();
    void sendAuthRequest();
    void sendActiveMessage();
    void sendSecureMessage(const std::string& receiver, const std::string& plaintext);
    
    // Métodos de processamento de respostas
    void processPtpResponse(const PtpResponse* ptpResponse);
    void processECDHResponse(const ECDHResponse* ecdhResponse);
    void processAuthGranted(const AuthGranted* authGranted);
    void processActiveMessage(const ActiveMessage* activeMessage);
    void processSecureMessage(const SecureMessage* msg);

    // Instâncias dos controladores e provedores de dados do veículo
    const traci::VehicleController* _mVehicleController = nullptr;
    const artery::VehicleDataProvider* _mVehicleDataProvider = nullptr;

    std::string _authgValue; // Valor de autenticação do gateway

    //statistics
    static const int VECTOR_SIZE = 100;
    int ecdhcount;
    double ecdh_times[VECTOR_SIZE];

    int ptpcount;
    double ptp_times[VECTOR_SIZE];   
    double ptp_net_times[VECTOR_SIZE];    //SIMULAÇÃO
    omnetpp::simtime_t ptpStartAt;                 

    int otp_count;
    double otp_times[VECTOR_SIZE];   
    double auth_net_times[VECTOR_SIZE];   // SIMUALÇÃO
    omnetpp::simtime_t authStartAt;                

    int gk_count;
    double gk_times[VECTOR_SIZE];    
    double gk_dist_times[VECTOR_SIZE];    

    
    int sec_enc_count;
    int sec_dec_count;
    double sec_enc_times[VECTOR_SIZE];    // Computation (Encrypt)
    double sec_dec_times[VECTOR_SIZE];    // Computation (Decrypt)
    double sec_net_times[VECTOR_SIZE];    // Network Latency (End-to-End)
    omnetpp::simtime_t ecdhStartAt;
    double ecdh_net_times[VECTOR_SIZE];

};

#endif // DEVICE_H_
