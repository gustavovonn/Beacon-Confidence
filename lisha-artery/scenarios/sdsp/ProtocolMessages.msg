packet BaseMessage {
    int type;
    string vehicleId;
    string SHA256_fw;
    double latitude;
    double longitude;
}

packet PtpRequest extends BaseMessage {
    string token;
    uint64 T1;
}

packet PtpResponse extends BaseMessage {
    string token;
    uint64 T1; 
    uint64 T2; 
    uint64 T3;
}
packet ActivationRequest extends BaseMessage {
    string CYg;
    double t0;
    double tf;
}

packet ActivationToken extends BaseMessage {
    double t0;
    double tf;
    string Authd;
    string Authg;
}

packet ECDHRequest extends BaseMessage {
    string Yd;
    string placeholder;
}

packet ECDHResponse extends BaseMessage {
    string Yg;
}

packet AuthRequest extends BaseMessage {
    string OTPd;
    string sharedSecret;
    string authd;
    string timestamp;
}

packet AuthGranted extends BaseMessage {
    string Authg;
    string OTPd;
    string KG;
    string CYg;
}

packet ActiveMessage extends BaseMessage {
    string Authh;
    string OTPpd;
    string Authde;
    string Authg;
}

packet FinalActivation extends BaseMessage {
    string UUIDd;
    string SHA256_fw;
}

packet SecureMessage extends BaseMessage {
    string encryptedData; 
    string authd;         
    string otpd;    
    simtime_t creationTime;      
    
}
