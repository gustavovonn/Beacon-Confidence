#include "Device.h"
#include <vanetza/btp/data_request.hpp>
#include "artery/tinycrypt/lib/include/tinycrypt/ecc_dh.h"
#include "artery/tinycrypt/lib/include/tinycrypt/ecc.h"
#include "artery/tinycrypt/lib/include/tinycrypt/hmac.h"
#include "artery/tinycrypt/lib/include/tinycrypt/sha256.h"
#include "artery/utility/Geometry.h"
#include "artery/utility/Identity.h"
#include "sdsp_msgs/ProtocolMessages_m.h"
#include "artery/arduinolibs/libraries/Crypto/Poly1305.h"
#include "messageTypes.h"
#include "utils.h"
#include <omnetpp.h>
#include <string>
#include <sstream>
#include <iomanip>
#include <map>
#include <ctime>


using namespace artery;
using namespace omnetpp;


Define_Module(Device);


void Device::initialize() {
    ItsG5Service::initialize();

    
    _mVehicleController = &getFacilities().get_const<traci::VehicleController>();
    _mVehicleDataProvider = &getFacilities().get_const<artery::VehicleDataProvider>();

    _authgValue = "AUTHD" + _mVehicleController->getVehicleId();

    std::string filePath = "/home/manu/SDSPV2X/artery/scenarios/sdsp/results/" 
                     + std::to_string(_mVehicleDataProvider->getStationId()) 
                     + "_KEY.txt";

    openandWritekey(generateRandomKey(), filePath);

    isAuthenticating = false;
    authStartTime = 0;
    kgRequestStartTime = 0;

    // sinais de medição
    authLatencySignal = registerSignal("authenticationLatency");
    v2vLatencySignal = registerSignal("v2vLatency");
    otpLatencySignal = registerSignal("otpLatency");
    ecdhLatencySignal = registerSignal("ecdhLatency");
    groupKeyLatencySignal = registerSignal("groupKeyLatency");
    aesEncryptLatencySignal = registerSignal("aesEncryptLatency");
    aesDecryptLatencySignal = registerSignal("aesDecryptLatency");

    
    ecdhcount = 0;
    ptpcount = 0;
    otp_count = 0;
    gk_count = 0;
    sec_enc_count = 0;
    sec_dec_count = 0;

    generateECDHKeys();
    //std::cout << "Device initialized: " << _authgValue << std::endl;
}

void Device::finish() {

    std::cout << " METRICS DEVICE " << _mVehicleController->getVehicleId() << std::endl;

    std::cout << "ECDH Latency: " << std::endl;    
    for (int i = 0; i < ecdhcount; i++)
    {
        std::cout << "\t" << i << "," << ecdh_times[i] << "\n";
    }

    std::cout << "ECDH Request Latency (simTime):" << std::endl;
    for (int i = 0; i < ecdhcount; i++)
    {
        std::cout << "\t" << i << "," << ecdh_net_times[i] << "\n";
    }

    std::cout << "PTP Latency (chrono):" << std::endl;
    for (int i = 0; i < ptpcount; i++) 
    {
        std::cout << "\t" << i << "," << ptp_times[i] << "\n";
    }

    std::cout << "PTP Request Latency (simTime):" << std::endl;
    for (int i = 0; i < ptpcount; i++) 
    {
        std::cout << "\t" << i << "," << ptp_net_times[i] << "\n";
    }

    std::cout << "OTP Latency (chrono):" << std::endl;
    for (int i = 0; i < otp_count; i++) 
    {
        std::cout << "\t" << i << "," << otp_times[i] << "\n";
    }

    std::cout << "Auth Request Latency (simTime):" << std::endl;
    for (int i = 0; i < otp_count; i++) 
    {
        std::cout << "\t" << i << "," << auth_net_times[i] << "\n";
    }

    std::cout << "Group Key Processing (chrono):" << std::endl;
    for (int i = 0; i < gk_count; i++) 
    {
        std::cout << "\t" << i << "," << gk_times[i] << "\n";
    }

    std::cout << "Group Key Distribution (simTime):" << std::endl;
    for (int i = 0; i < gk_count; i++) 
    {
        std::cout << "\t" << i << "," << gk_dist_times[i] << "\n";
    }

    std::cout << "Sec Msg Encrypt (chrono):" << std::endl;
    for (int i = 0; i < sec_enc_count; i++) 
    {
        std::cout << "\t" << i << "," << sec_enc_times[i] << "\n";
    }

    std::cout << "Sec Msg Decrypt (chrono):" << std::endl;
    for (int i = 0; i < sec_dec_count; i++) 
    {
        std::cout << "\t" << i << "," << sec_dec_times[i] << "\n";
    }

    std::cout << "Sec Msg Net Latency (simTime):" << std::endl;
    for (int i = 0; i < sec_dec_count; i++) 
    {
        std::cout << "\t" << i << "," << sec_net_times[i] << "\n";
    }
    
    std::cout << "Device Finished; " << std::endl;
}

void Device::generateECDHKeys() {
   if (uECC_make_key(devicePublicKey, devicePrivateKey, uECC_secp256r1())) {
      
       //std::cout << "Device Public Key: ";
       for (int i = 0; i < 64; i++) {
          // std::cout << std::hex << std::setw(2) << (int)devicePublicKey[i];
       }
   }
}


// Em Device.cc


void Device::trigger() {
   Enter_Method("Device trigger");


   // Parte 1: Lógica de Sincronização de Tempo (PTP)
   // Se o tempo desde a última sincronização for maior que o timeout, marca como não sincronizado.
   if (simTime() - lastSyncTime > syncTimeout) {
       isTimeSynced = false;
   }


   // Se não estiver sincronizado, envia um novo pedido de PTP.
   if (!isTimeSynced && (simTime() - lastPtpRequestTime < AUTH_TIMEOUT_S)) {
      // std::cout << "Sending time sync, vehicle id: " << _mVehicleController->getVehicleId() << std::endl;
       lastPtpRequestTime = simTime();
       sendPtpRequest();
       return; // Retorna para esperar a resposta do PTP
   }


   // Parte 2: Lógica de Troca de Chaves (ECDH)
   // Se estiver sincronizado mas a troca de chaves ainda não foi feita, inicia o ECDH.
    //if (isTimeSynced && !ecdhComplete && !ecdhRequested) {
     //   ecdhRequested = true;
        //sendECDHRequest();
   // }


   // Parte 3: Lógica de Envio da Mensagem Segura (com Atraso Fixo)
   // Apenas o veículo 0.0 executará esta lógica.
   if (isAuthorized && _mVehicleController->getVehicleId() == "0.0" && !messageSent) {
      
       
       // Não envia a mensagem antes dar tempo
       // ao veículo 0.1 de se autenticar.
       if (simTime() > 10.0) {
           //std::cout << "Veiculo 0.0 (autorizado) enviando SecureMessage para 0.1..." << std::endl;
           sendSecureMessage("1.0", "transito");
           messageSent = true;
       }
   }
}


void Device::sendPtpRequest() {
    if (!isAuthenticating && !isAuthorized) {
        authStartTime = simTime();
        isAuthenticating = true;
    }

   auto start_c = std::chrono::high_resolution_clock::now();
   auto packet = new PtpRequest();
   packet->setT1(simTime().inUnit(SIMTIME_US));
   packet->setType(PTP_REQUEST);
   visitAuthPacketSender(packet, _mVehicleDataProvider);
   packet->setVehicleId(_mVehicleController->getVehicleId().c_str());
   packet->setByteLength(40);
   ptpStartAt = simTime();

   vanetza::btp::DataRequestB req = createDataRequestB<Device>(getPortNumber());
   req.gn.traffic_class = vanetza::geonet::TrafficClass(0); 

    // Fix: Set Transport Type to Single Hop Broadcast (SHB)
    req.gn.transport_type = vanetza::geonet::TransportType::SHB;

   request(req, packet);

    auto end_c = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> dur = end_c - start_c;

        if (ptpcount < VECTOR_SIZE) {
            ptp_times[ptpcount] = dur.count();
            
        }


  // std::cout << "Sent PTP Request from " << packet->getVehicleId() << std::endl;
}


void Device::sendECDHRequest() {
   auto start_c = std::chrono::high_resolution_clock::now();
   auto ecdhRequest = new ECDHRequest();
   ecdhRequest->setType(ECDH_REQUEST);
   ecdhRequest->setVehicleId(_mVehicleController->getVehicleId().c_str());


   std::ostringstream os;
   for (int i = 0; i < sizeof(devicePublicKey); i++) {
       os << std::hex << std::setw(2) << std::setfill('0') << (int)devicePublicKey[i];
   }
   std::string devicePublicKeyHex = os.str();


   ecdhRequest->setYd(devicePublicKeyHex.c_str());
   ecdhRequest->setByteLength(40);
   
   ecdhStartAt = simTime(); 
   vanetza::btp::DataRequestB req = createDataRequestB<Device>(getPortNumber());
   req.gn.traffic_class = vanetza::geonet::TrafficClass(0); 

    // Fix: Set Transport Type to Single Hop Broadcast (SHB)
    req.gn.transport_type = vanetza::geonet::TransportType::SHB;

   request(req, ecdhRequest);
   auto end_c = std::chrono::high_resolution_clock::now();

   std::chrono::duration<double> duration_s = end_c - start_c;

        if (ecdhcount < VECTOR_SIZE ) {
            ecdh_times[ecdhcount] = duration_s.count();
            
       
        } 
  // std::cout << "Sent ECDHRequest from " << ecdhRequest->getVehicleId() << std::endl;
   //std::cout << "Device Public Key (send): " << devicePublicKeyHex << std::endl;
}


void Device::sendAuthRequest() {
    auto start_c = std::chrono::high_resolution_clock::now();

    auto packet = new AuthRequest();
    packet->setType(AUTH_REQUEST);
    visitAuthPacketSender(packet, _mVehicleDataProvider);
    packet->setVehicleId(_mVehicleController->getVehicleId().c_str());

    // Simulated firmware hash
    std::string firmwareHash = "firmware_hash";
    packet->setSHA256_fw(firmwareHash.c_str());

    // Prepare OTP components
    std::string key(reinterpret_cast<char*>(sharedSecret), sizeof(sharedSecret));
    std::string auth = "AUTHD" + _mVehicleController->getVehicleId();
    std::string message = "MessageData";
    std::string fullTimestamp = std::to_string(adjustedTime.inUnit(omnetpp::SIMTIME_US)/100000);


    std::string concatenatedString = auth + message + fullTimestamp;
    std::string timestamp = fullTimestamp;
    
    // simtime_t start = simTime();

    auto start_c1 = std::chrono::high_resolution_clock::now();

    std::string calculatedOtp = calculateOTP(key, concatenatedString);

    
    auto end_c1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> dur = end_c1 - start_c1;

    // simtime_t end = simTime();
    // emit(otpLatencySignal, end - start);

    otp = calculatedOtp;

    //std::cout << "[OTP] Timestamp completo (Device): " << fullTimestamp << std::endl;
    //std::cout << "[OTP] Timestamp truncado (Device): " << timestamp << std::endl;
   // std::cout << "Generated OTP (Device): " << calculatedOtp << std::endl;

    packet->setTimestamp(timestamp.c_str());
    packet->setOTPd(calculatedOtp .c_str());//substr(0, 6).c_str());
    packet->setSharedSecret(key.c_str());
    packet->setByteLength(40);

    // Salva o tempo para calcular latência até receber a Group Key (Pode manter para lógica interna antiga)
    kgRequestStartTime = simTime();


    authStartAt = simTime();

    vanetza::btp::DataRequestB req = createDataRequestB<Device>(getPortNumber());
    

    // Fix: Access traffic_class inside the 'gn' struct and set ID to 0 (High Priority)
    req.gn.traffic_class = vanetza::geonet::TrafficClass(0); 

    // Fix: Set Transport Type to Single Hop Broadcast (SHB)
    req.gn.transport_type = vanetza::geonet::TransportType::SHB;    

    request(req, packet);
    auto end_c = std::chrono::high_resolution_clock::now();
    
    if (otp_count < VECTOR_SIZE) {
        otp_times[otp_count] = dur.count();
    }

    dur = end_c - start_c;

    if (gk_count < VECTOR_SIZE) {
        gk_times[gk_count] = dur.count();
    }

   // std::cout << "Sent AuthRequest from " << packet->getVehicleId() << std::endl;
}



void Device::sendSecureMessage(const std::string& receiver, const std::string& plaintext) {
    auto packet = new SecureMessage();
    packet->setType(SECURE_MESSAGE);
    packet->setVehicleId(receiver.c_str());
    packet->setAuthd(_authgValue.c_str());
    packet->setOtpd(otp.substr(0, 6).c_str());
    

    packet->setCreationTime(simTime()); // timestamp de criação

    std::string fullKey(reinterpret_cast<char*>(groupKey), sizeof(groupKey));
    std::string aesKey = sha256(fullKey).substr(0, 16);

    // --- MEDIÇÃO DE TEMPO DE CRIPTOGRAFIA ---
   // simtime_t start = simTime();
    auto start_c = std::chrono::high_resolution_clock::now();
    std::string encrypted = encryptMessage(aesKey, plaintext);
    auto end_c = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> dur = end_c - start_c;
    if (sec_enc_count < VECTOR_SIZE) {
        sec_enc_times[sec_enc_count++] = dur.count();
    }
   // simtime_t end = simTime();
   // emit(aesEncryptLatencySignal, end - start);

    packet->setEncryptedData(encrypted.c_str());
    packet->setByteLength(150);

    auto req = createDataRequestB<Device>(getPortNumber());
    request(req, packet);

   // std::cout << "Veiculo " << _mVehicleController->getVehicleId()
           //   << " enviou SecureMessage para " << receiver << "." << std::endl;
}


void Device::indicate(const vanetza::btp::DataIndication& ind, omnetpp::cPacket* packet) {
   Enter_Method("Device indicate");
   auto msg = check_and_cast<const BaseMessage*>(packet);




   switch (msg->getType()) {
       case PTP_RESPONSE:
           processPtpResponse(check_and_cast<const PtpResponse*>(msg));
           if(isTimeSynced && !ecdhRequested){
                sendECDHRequest();
                ecdhRequested = true;
           }

           break;
       case ECDH_RESPONSE:
           processECDHResponse(check_and_cast<const ECDHResponse*>(msg));
           if (ecdhComplete && !authrequested )
           {
                sendAuthRequest();
                authrequested =true;
           }
           
           break;
       case AUTH_GRANTED:
           processAuthGranted(check_and_cast<const AuthGranted*>(msg));
           break;
       case ACTIVE_MESSAGE:
           processActiveMessage(check_and_cast<const ActiveMessage*>(msg));
           break;


           case SECURE_MESSAGE:
                if (isAuthorized) {
                   processSecureMessage(check_and_cast<const SecureMessage*>(msg));
           } else {
               //std::cout << "Device " << _mVehicleController->getVehicleId()
                     //    << " received SECURE_MESSAGE but is not authorized." << std::endl;
            }
            break;
      
          


       default:
  
           break;
      

      
   }

   delete msg;
}


   void Device::processPtpResponse(const PtpResponse* ptpResponse) {
        if (ptpResponse->getVehicleId() != _mVehicleController->getVehicleId()) return;
       
        if (ptpcount < VECTOR_SIZE) {
            ptp_net_times[ptpcount] = (simTime() - ptpStartAt).dbl();
        }

        auto start_c = std::chrono::high_resolution_clock::now();
  
       int64_t T1 = ptpResponse->getT1();
       int64_t T2 = ptpResponse->getT2();
       int64_t T3 = ptpResponse->getT3();
       int64_t T4 = simTime().inUnit(omnetpp::SIMTIME_US);
  
       int64_t offset = (T2 - T1 + T3 - T4) / 2;
       int64_t delay = (T4 - T1 - (T3 - T2)) / 2;
      
       adjustedTime = simTime() + omnetpp::SimTime(offset, omnetpp::SIMTIME_US);
       isTimeSynced = true;
       lastSyncTime = simTime();
       auto end_c = std::chrono::high_resolution_clock::now();
       std::chrono::duration<double> dur = end_c - start_c;

        if (ptpcount < VECTOR_SIZE) {
            ptp_times[ptpcount] += dur.count();
            ptpcount++; 
        }

       uint64_t time_us = static_cast<uint64_t>(adjustedTime.inUnit(omnetpp::SIMTIME_US));
       std::string fullTimestamp = std::to_string(time_us);
       std::string timestamp = fullTimestamp.substr(0, 6);


       //std::cout << "[OTP] Timestamp completo (Device): " << fullTimestamp << std::endl;
      // std::cout << "[OTP] Timestamp truncado (Device): " << timestamp << std::endl;


     
      // std::cout << "Clocks sincronizados! Offset (ms): "
       //          << offset / 1000.0 
         //        << " | Delay (ms): "
         //        << delay / 1000.0  
         //        << std::endl;
  
      // std::ostringstream oss;
      // oss << "Clocks sincronizados! Offset (ms): "
         //  << offset / 1000.0
        //   << " | Delay (ms): "
         //  << delay / 1000.0;
  
     //  std::string message = oss.str();
   }


   void Device::processECDHResponse(const ECDHResponse* ecdhResponse) {
    if (ecdhResponse->getVehicleId() != _mVehicleController->getVehicleId() && ecdhComplete) {
        return;
    }
    auto start_real = std::chrono::high_resolution_clock::now();
    std::string gatewayPublicKeyHex = ecdhResponse->getYg();
    
     //std::cout << "Gateway Public Key (Received as Hex): " << gatewayPublicKeyHex << std::endl;

    std::vector<uint8_t> gatewayPublicKeyVec = hexToBytes(gatewayPublicKeyHex);
    if (gatewayPublicKeyVec.size() != 64) {
        std::cerr << "Error: Received gateway public key has incorrect size." << std::endl;
        return;
    }

    // --- MEDIÇÃO DE TEMPO ECDH ---
    
    bool ok = uECC_shared_secret(gatewayPublicKeyVec.data(), devicePrivateKey, sharedSecret, uECC_secp256r1());
    auto end_real = std::chrono::high_resolution_clock::now();

    if (ok) {
        // Calcula a duração em segundos, usando double para alta precisão
        std::chrono::duration<double> duration_s = end_real - start_real;

        // Emite o valor em segundos (simtime_t é um double) para o arquivo .vec
        if (ecdhcount < VECTOR_SIZE ) {
            ecdh_times[ecdhcount] += duration_s.count();
            ecdh_net_times[ecdhcount] = (simTime() - ecdhStartAt).dbl();
            ecdhcount++;
        //emit(ecdhLatencySignal, (simtime_t) duration_s.count());
        // Imprime o valor de alta precisão no log para o script Python
        //std::cout << "[SDSP_METRIC] ecdhLatency=" << duration_s.count() << std::endl;
        }
        ecdhComplete=true;
    }else {
        //std::cerr << "Failed to calculate shared secret on device." << std::endl;
    }
}



void Device::processAuthGranted(const AuthGranted* authGranted) {
    if (authGranted->getVehicleId() != _mVehicleController->getVehicleId()) {
        
        return;
    }


    // Calcula o tempo desde o envio do AuthRequest até a chegada do AuthGranted
    if (otp_count < VECTOR_SIZE) {/*  */
        auth_net_times[otp_count] = (simTime() - authStartAt).dbl();
    }
     auto start_c = std::chrono::high_resolution_clock::now();

    
    // Calcula o tempo desde que o Gateway criou a mensagem até chegar aqui
    if (gk_count < VECTOR_SIZE) {
        gk_dist_times[gk_count] = (simTime() - authGranted->getCreationTime()).dbl();
    }

    std::string groupKeyHex = authGranted->getKG();
    std::vector<uint8_t> groupKeyVec = hexToBytes(groupKeyHex);

    if(groupKeyVec.size() == sizeof(this->groupKey)) {
        
        
        // simtime_t start = simTime();

        // --- ADICIONAR (Início Chrono: Group Key Processing) ---
       

        std::memcpy(this->groupKey, groupKeyVec.data(), sizeof(this->groupKey));
        std::string hexKey = bytesToHex(this->groupKey, 16);
        openandWritekey(hexKey,"/home/manu/SDSPV2X/artery/scenarios/sdsp/results/" + std::to_string(_mVehicleDataProvider->getStationId()) + "_KEY.txt");
        
        // --- ADICIONAR (Fim Chrono: Group Key Processing) ---
        auto end_c = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> dur = end_c - start_c;

        if (gk_count < VECTOR_SIZE) {
            gk_times[gk_count] += dur.count();
            gk_count++;
        }

        // simtime_t end = simTime();
        // emit(groupKeyLatencySignal, end - start);

        
        otp_count++;

        // Latência total (Lógica antiga mantida opcionalmente ou comentada)
        // simtime_t totalKgLatency = simTime() - kgRequestStartTime;
        // emit(groupKeyLatencySignal, totalKgLatency);

        //std::cout << "Group Key (KG) received and stored." << std::endl;
    } else {
        std::cerr << "Error: Received Group Key has incorrect size." << std::endl;
        return;
    }

    isAuthorized = true;

    // Lógica antiga de sinalização de latência total
    if (isAuthenticating) {
        simtime_t latency = simTime() - authStartTime;
        // emit(authLatencySignal, latency);
        emit(authLatencySignal, latency);
        //std::cout << "[SDSP_METRIC] authenticationLatency=" << latency << std::endl;
        isAuthenticating = false;
    }

    this->_authgValue = authGranted->getAuthg();
   // std::cout << "Authorization granted for " << authGranted->getVehicleId() << std::endl;

    sendActiveMessage();
}


void Device::sendActiveMessage() {
   auto packet = new ActiveMessage();
   packet->setType(ACTIVE_MESSAGE);
   packet->setVehicleId(_mVehicleController->getVehicleId().c_str());


   std::string authToken = _authgValue;
   //std::string otp = ;
   std::string authValue = "VALID_AUTHD_1";


   packet->setAuthh(authToken.c_str());
   packet->setOTPpd(otp.c_str());
   packet->setAuthde(authValue.c_str());
   packet->setByteLength(40);


   vanetza::btp::DataRequestB req = createDataRequestB<Device>(getPortNumber());
   req.gn.traffic_class = vanetza::geonet::TrafficClass(0); 

    // Fix: Set Transport Type to Single Hop Broadcast (SHB)
    req.gn.transport_type = vanetza::geonet::TransportType::SHB;

   request(req, packet);


  // std::cout << "Sent ActiveMessage from " << packet->getVehicleId() << std::endl;
}


void Device::processActiveMessage(const ActiveMessage* activeMessage) {
 //  std::cout << "Processing active message from vehicle id: " << activeMessage->getVehicleId() << std::endl;
}


void Device::processSecureMessage(const SecureMessage* msg) {
    std::string receiverId = std::string(msg->getVehicleId());
   
    // --- LÓGICA DE VERIFICAÇÃO ---
    if (_mVehicleController->getVehicleId() != receiverId) {
        return; // Ignora se a mensagem não for para este veículo
    }

    // --- ADICIONAR (Rede: Secure Message Latency - End-to-End) ---
    // Mede o tempo total desde o envio (CreationTime) até o processamento aqui
    if (sec_dec_count < VECTOR_SIZE) {
        sec_net_times[sec_dec_count] = (simTime() - msg->getCreationTime()).dbl();
    }
   
  //  std::cout << "--- DEVICE " << _mVehicleController->getVehicleId()
   //           << " recebeu SecureMessage destinada a ele. ---" << std::endl;
   // std::cout << "Remetente (Authd): " << msg->getAuthd() << std::endl;
 
    std::string fullKey(reinterpret_cast<const char*>(groupKey), sizeof(groupKey));
    std::string aesKey = sha256(fullKey).substr(0, 16);
    std::string decrypted;
 
    try {
        
        // simtime_t start = simTime();
        
        auto start_c = std::chrono::high_resolution_clock::now();

        decrypted = decryptMessage(aesKey, msg->getEncryptedData());
        
        auto end_c = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> dur = end_c - start_c;

        if (sec_dec_count < VECTOR_SIZE) {
            sec_dec_times[sec_dec_count] = dur.count();
           
            sec_dec_count++;
        }

        // --- COMENTAR (Antiga medição de latência local) ---
        // simtime_t end = simTime();
        // emit(aesDecryptLatencySignal, end - start);

       // std::cout << ">>> MENSAGEM DESCRIPTOGRAFADA COM SUCESSO! <<<" << std::endl;
       // std::cout << "Conteudo: '" << decrypted << "'" << std::endl;

      
        simtime_t v2v_latency = simTime() - msg->getCreationTime();
        emit(v2vLatencySignal, v2v_latency);
       // std::cout << "[SDSP_METRIC] v2vLatency=" << v2v_latency << std::endl;        

    } catch (const std::exception& e) {
        std::cerr << "ERRO: Falha ao descriptografar a mensagem no veiculo "
                  << _mVehicleController->getVehicleId() << ". Excecao: " << e.what() << std::endl;
    }
}





