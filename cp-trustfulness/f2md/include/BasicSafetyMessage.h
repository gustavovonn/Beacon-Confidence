#pragma once

#include "veins/base/utils/Coord.h"  // Coord mock
#include "omnetpp.h"

namespace LAddress {
    typedef long L2Type; 
}

class BasicSafetyMessage {
public:
    veins::Coord pos, posConf, speed, speedConf;
    veins::Coord heading, headingConf, accel, accelConf;
    double width = 2.0;
    double length = 4.5;
    double arrivalTime = 0.0;
    double sendingTime = 0.0;
    unsigned long pseudonym = 0;
    int mbType = 0;       
    int attackType = 0;   
    
    // Observer / Receiver context for offline plausibility evaluation
    veins::Coord recv_pos;
    veins::Coord recv_speed;
    veins::Coord recv_heading;
    long receiver_id = 0;
    long receiver_pseudo = 0;

    veins::Coord& getSenderPos() { return pos; }
    veins::Coord& getSenderPosConfidence() { return posConf; }
    veins::Coord& getSenderSpeed() { return speed; }
    veins::Coord& getSenderSpeedConfidence() { return speedConf; }
    veins::Coord& getSenderHeading() { return heading; }
    veins::Coord& getSenderHeadingConfidence() { return headingConf; }
    veins::Coord& getSenderAccel() { return accel; }
    veins::Coord& getSenderAccelConfidence() { return accelConf; }
    
    double getSenderWidth() const { return width; }
    double getSenderLength() const { return length; }
    unsigned long getSenderPseudonym() const { return pseudonym; }

    simtime_t getArrivalTime() const { return simtime_t(arrivalTime); }
    simtime_t getSendingTime() const { return simtime_t(sendingTime); }
    
    int getSenderMbType() const { return mbType; }
    int getSenderAttackType() const { return attackType; }
};
