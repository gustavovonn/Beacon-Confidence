#include "utils.h"
#include "artery/arduinolibs/libraries/Crypto/AES.h"
#include "artery/tinycrypt/lib/include/tinycrypt/hmac.h"
#include "artery/tinycrypt/lib/include/tinycrypt/sha256.h"
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/simtime.h>
#include <vanetza/asn1/its/Latitude.h>
#include <vanetza/asn1/its/Longitude.h>
#include <vanetza/btp/data_request.hpp>
#include <vanetza/geonet/interface.hpp>
#include <iomanip>
#include <random>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <vanetza/geonet/data_request.hpp>
#include <vanetza/geonet/areas.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/length.hpp>
#include <chrono>
#include <omnetpp.h>
#include <fstream>



std::string bytesToHex(const uint8_t* bytes, size_t len)
{
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (size_t i = 0; i < len; ++i) {
        ss << std::setw(2) << static_cast<int>(bytes[i]);
    }
    return ss.str();
}

std::vector<uint8_t> hexToBytes(const std::string& hex)
{
    std::vector<uint8_t> bytes;
    if (hex.length() % 2!= 0) {
        throw std::invalid_argument("Hex string must have an even number of characters.");
    }
    for (unsigned int i = 0; i < hex.length(); i += 2) {
        std::string byteString = hex.substr(i, 2);
        bytes.push_back(static_cast<uint8_t>(strtol(byteString.c_str(), nullptr, 16)));
    }
    return bytes;
}

std::string encryptMessage(const std::string& key, const std::string& plaintext)
{
    // --- CORREÇÃO E COMPLEMENTO DA MEDIÇÃO DE TEMPO ---
    auto start = std::chrono::high_resolution_clock::now(); // 1. Inicia o cronómetro aqui

    if (key.length() != 16) {
        throw std::invalid_argument("Encryption key must be 16 bytes for AES-128.");
    }

    AES128 aes;
    aes.setKey(reinterpret_cast<const uint8_t*>(key.data()), 16);

    std::vector<uint8_t> iv(16);
    std::random_device rd;
    std::generate(iv.begin(), iv.end(), [&]() { return rd(); });

    size_t padding_len = 16 - (plaintext.length() % 16);
    std::string padded_plaintext = plaintext;
    padded_plaintext.append(padding_len, static_cast<char>(padding_len));

    std::vector<uint8_t> ciphertext(padded_plaintext.length());
    const uint8_t* prev_block = iv.data();

    for (size_t i = 0; i < padded_plaintext.length(); i += 16) {
        uint8_t block_to_encrypt[16]; 
        for (size_t j = 0; j < 16; ++j) {
            block_to_encrypt[j] = padded_plaintext[i + j] ^ prev_block[j];
        }
        aes.encryptBlock(ciphertext.data() + i, block_to_encrypt);
        prev_block = ciphertext.data() + i;
    }
    
    std::string result(iv.begin(), iv.end());
    result.append(reinterpret_cast<const char*>(ciphertext.data()), ciphertext.size());

    auto stop = std::chrono::high_resolution_clock::now(); // 2. Para o cronómetro
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    // 3. Grava o resultado como um escalar no OMNeT++
    if (omnetpp::getSimulation()->getContextModule()) {
        omnetpp::getSimulation()->getContextModule()->recordScalar("encryptionTime_us", duration.count());
    }
    

    return result;
}

std::string decryptMessage(const std::string& key, const std::string& encryptedMessage)
{
    if (key.length()!= 16) {
        throw std::invalid_argument("Decryption key must be 16 bytes for AES-128.");
    }
    if (encryptedMessage.length() < 16) {
        throw std::invalid_argument("Encrypted message is too short to contain an IV.");
    }

    AES128 aes;
    aes.setKey(reinterpret_cast<const uint8_t*>(key.data()), 16);

    std::vector<uint8_t> iv(encryptedMessage.begin(), encryptedMessage.begin() + 16);
    std::string ciphertext_str = encryptedMessage.substr(16);

    if (ciphertext_str.length() % 16!= 0) {
        throw std::invalid_argument("Ciphertext length is not a multiple of the block size.");
    }

    std::vector<uint8_t> decrypted_text(ciphertext_str.length());
    const uint8_t* prev_block = iv.data();

    for (size_t i = 0; i < ciphertext_str.length(); i += 16) {
        uint8_t block_to_decrypt[16];
        std::memcpy(block_to_decrypt, ciphertext_str.data() + i, 16);

        aes.decryptBlock(decrypted_text.data() + i, block_to_decrypt);

        for (size_t j = 0; j < 16; ++j) {
            decrypted_text[i + j] ^= prev_block[j];
        }
        prev_block = block_to_decrypt;
    }

    size_t padding_len = decrypted_text.back();
    if (padding_len > 16 || padding_len == 0) { 
        throw std::runtime_error("Invalid padding value during decryption.");
    }
    for(size_t i = decrypted_text.size() - padding_len; i < decrypted_text.size(); ++i) {
        if (decrypted_text[i]!= padding_len) {
            throw std::runtime_error("Invalid padding bytes during decryption.");
        }
    }

    return std::string(decrypted_text.begin(), decrypted_text.end() - padding_len);
}

std::string calculateHMACSHA256(const std::string& key, const std::string& data)
{
    uint8_t digest[32]; 
    tc_hmac_state_struct hmacState;

    tc_hmac_set_key(&hmacState, reinterpret_cast<const uint8_t*>(key.data()), key.size());
    tc_hmac_init(&hmacState);
    tc_hmac_update(&hmacState, reinterpret_cast<const uint8_t*>(data.data()), data.size());
    tc_hmac_final(digest, sizeof(digest), &hmacState);

    return bytesToHex(digest, sizeof(digest));
}

std::string sha256(const std::string& data)
{
    uint8_t digest[32];
    tc_sha256_state_struct shaState;

    if (tc_sha256_init(&shaState) != 1) {
        throw std::runtime_error("tc_sha256_init failed");
    }
    tc_sha256_update(&shaState, reinterpret_cast<const uint8_t*>(data.data()), data.size());
    tc_sha256_final(digest, &shaState);

    return bytesToHex(digest, sizeof(digest));
}

std::string calculateOTP(const std::string& key, const std::string& data)
{
    return calculateHMACSHA256(key, data);
}

bool verifyOTP(const std::string& otp, const std::string& key, const std::string& data)
{
    std::string expected_otp = calculateOTP(key, data);
    
    return otp == expected_otp;
}

void generateGroupKey(uint8_t* key, size_t len)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist(0, 255);

    for (size_t i = 0; i < len; ++i) {
        key[i] = static_cast<uint8_t>(dist(gen));
    }
}

bool openandWritekey(const std::string& key, const std::string& filename)
{
    std::ofstream file(filename, std::ios::out | std::ios::trunc);
    if (!file.is_open()) {
        return false;
    }

    file << key << std::endl;
    file.close();

    return true;
}

std::string readKey(const std::string& vehicleId)
{
    const std::string filename = vehicleId + "_KEY.txt";
    std::ifstream file(filename);

    if (!file.is_open()) {
        return ""; // erro ao abrir
    }

    std::string key;
    std::getline(file, key);
    file.close();

    return key; // chave AES (ex: hex)
}

std::string generateRandomKey()
{
    const size_t KEY_SIZE = 16; // 16 bytes = 128 bits

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(0, 255);

    std::ostringstream hexKey;

    for (size_t i = 0; i < KEY_SIZE; ++i) {
        int byte = dist(gen);
        hexKey << std::hex << std::setw(2) << std::setfill('0') << byte;
    }

    return hexKey.str(); // 32 caracteres hex (AES-128)
}

void visitAuthPacketSender(BaseMessage* packet, const artery::VehicleDataProvider* vehicleDataProvider)
{
    if (!vehicleDataProvider) {
        return;
    }




   
    double lat = round(vehicleDataProvider->latitude(), MICRODEGREE) * Longitude_oneMicrodegreeEast * (M_PI / 180.0);
    double lon = round(vehicleDataProvider->longitude(), MICRODEGREE) * Latitude_oneMicrodegreeNorth * (M_PI / 180.0);

    packet->setLatitude(lat);
    packet->setLongitude(lon);
}






