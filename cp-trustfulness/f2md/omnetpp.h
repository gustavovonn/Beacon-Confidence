#pragma once

#include <iostream>
#include <string>
#include <cmath>
#include <cassert>

// 1. Mock Logging
#define EV std::cout
#define EV_INFO std::cout
#define EV_WARN std::cout
#define EV_ERROR std::cerr
#define EV_DEBUG std::cout
#define EV_FATAL std::cerr
#define EV_TRACE std::cout

// 2. Mock simtime_t so .dbl() and math operations work natively
class simtime_t {
    double val;
public:
    simtime_t(double v = 0.0) : val(v) {}
    double dbl() const { return val; }
    
    bool operator<(const simtime_t& o) const { return val < o.val; }
    bool operator>(const simtime_t& o) const { return val > o.val; }
    bool operator<=(const simtime_t& o) const { return val <= o.val; }
    bool operator>=(const simtime_t& o) const { return val >= o.val; }
    bool operator==(const simtime_t& o) const { return val == o.val; }
    bool operator!=(const simtime_t& o) const { return val != o.val; }
    
    simtime_t operator-(const simtime_t& o) const { return simtime_t(val - o.val); }
    simtime_t operator+(const simtime_t& o) const { return simtime_t(val + o.val); }
};

inline simtime_t simTime() { return simtime_t(0.0); }

// 3. Mock OMNeT++ classes (cObject, cOutVector, etc.)
namespace omnetpp {
    class cObject {
    public:
        virtual ~cObject() {}
        virtual const char* getClassName() const { return "cObject"; }
    };
    class cMessage : public cObject {};
    
    // Mock for Statistics
    class cOutVector : public cObject {
    public:
        void setName(const char* name) {}
        void record(double value) {}
    };
    
    class cStdDev : public cObject {
    public:
        void setName(const char* name) {}
        void collect(double value) {}
        double getMean() const { return 0.0; }
        long getCount() const { return 0; }
    };

    inline void error(const char* msg) { 
        std::cerr << "OMNeT++ ERROR: " << msg << "\n"; 
    }
}

using namespace omnetpp;