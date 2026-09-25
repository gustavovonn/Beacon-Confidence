#ifndef EGOMOTIONCONTROLLER_H_
#define EGOMOTIONCONTROLLER_H_

#include "traci/API.h"
#include "artery/application/ItsG5Service.h"
#include "motion_vector.h"
#include <iostream>
#include "json.hpp"
#include <vector>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
using json = nlohmann::json;

// forward declaration
namespace traci { class VehicleController; }

class EgoMotionController : public artery::ItsG5Service
{
    public:
        static const bool BINARY_SAVE = false;
        static constexpr double INTEGER_FLOAT_PRECISION = 10000000.0;
        static constexpr double FROM_MICRO_TO_SECONDS = 1/1000000.0;
        static const bool DEBUG = true;
        static const bool HYSTERICALLY_DEBUG = DEBUG && false;

        enum commands { END_CONFIG=0,
                        SET_POS=1,
                        SET_TARGET=2,
                        SET_REPORT_MV=3,
                        SET_ROUTE=4,
                        SET_ROUTE_SPEED=5
                        };

    public:
        void trigger() override;
        std::vector<std::string> gen_route(json data);
        virtual ~EgoMotionController();

    protected:
        void initialize() override;

    private:
        bool speed_route_set;
        bool config_done;
        bool report_mv;
        unsigned int consumer_index;
        bool setup_control();
        void write_pipe(std::string data);
        std::string read_pipe();
        const traci::VehicleController* _mVehicleController;
        const artery::VehicleDataProvider* _mVehicleDataProvider = nullptr;
        std::vector<std::string> _route;
        json _speed_data;
        bool _route_is_set;
        std::string controlinput;
        std::string controloutput;
        int controlinputid;
        int controloutputid;
};

#endif /* EGOMOTIONCONTROLLER_H_ */
