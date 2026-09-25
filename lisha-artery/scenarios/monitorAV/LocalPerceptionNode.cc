#include "LocalPerceptionNode.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/CaObject.h"
#include <math.h> 
#include <fstream>
#include <boost/math/constants/constants.hpp>
#include <boost/units/cmath.hpp>
#include <boost/units/quantity.hpp>
#include "artery/traci/VehicleController.h"
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/cconfiguration.h>
#include <sys/stat.h>

#define BOOST_GEOMETRY_DISABLE_DEPRECATED_03_WARNING
#define TRACK_SIZE 10
#define DIST_THRESHOLD 3.0
#define SECOND_MS 1000000

using namespace omnetpp;

auto microdegree5 = vanetza::units::degree * boost::units::si::micro;

# define M_PI           3.14159265358979323846  /* pi */

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}

Define_Module(LocalPerceptionNode)

LocalPerceptionNode::~LocalPerceptionNode()
{
    // cancelAndDelete(mTrigger);
}

void LocalPerceptionNode::initialize() {
    artery::ItsG5BaseService::initialize();
    _mVehicleController = &getFacilities().get_const<traci::VehicleController>();
    _mVehicleDataProvider = &getFacilities().get_const<artery::VehicleDataProvider>();
    // localEnvmod = &getFacilities().get_const<artery::LocalEnvironmentModel>();

    
    
    const auto cfg = getSimulation()->getEnvir()->getConfigEx();
    const auto cfg_run_number = cfg->getVariable("runnumber");
    runnumber= std::string(cfg_run_number);

    std::string folder("results/results_"+runnumber);

    const int dir_err = mkdir(folder.c_str() , S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    

    std::ofstream myfile;
    myfile.open("results/results_"+runnumber+"/localperception-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::trunc);
    myfile.close();

    myfile.open("results/results_"+runnumber+"/localperceptionGT-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::trunc);
    myfile.close();

    myfile.open("results/results_"+runnumber+"/oldcam-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::trunc);
    myfile.close();

    myfile.open("results/results_"+runnumber+"/oldcpm-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::trunc);
    myfile.close();

    myfile.open("results/results_"+runnumber+"/uniqueids-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::trunc);
    myfile.close();

    getFacilities().register_mutable(this);
}

void LocalPerceptionNode::trigger() {
    // std::cout<<"id: "<<_mVehicleDataProvider->station_id()<<"\n";
    current_ts=_mVehicleDataProvider->updated().inUnit(omnetpp::SIMTIME_US)-100000;
    if(rawPerceptionData.size()>TRACK_SIZE){
        registerOldestGT();
        removeOldest(rawPerceptionData);
    }
    if(trackedObjectData.size()>TRACK_SIZE){
        // registerOldest(trackedObjectData);
        removeOldest(trackedObjectData);
    }
    if(!rawPerceptionData.empty()){
        // std::cout<<"\n start: "<<_mVehicleDataProvider->updated()<<" current_ts:"<<current_ts<<"\n";
        // for(auto iter=rawPerceptionData.begin(); iter!=rawPerceptionData.end(); iter++){
        //     std::cout<<"time: "<<iter->first<<"\n";
        // }
        objectAssociation();
        dataFusion();
        pruneOldObjects();
    }
}

std::vector<tracked_object> LocalPerceptionNode::getTrackedObjects(){
    return trackedObjects;
}

void LocalPerceptionNode::pruneOldObjects(){
    for(auto iter=trackedObjects.begin(); iter!=trackedObjects.end(); iter++){
        long delta = current_ts-iter->fusionUKF.time_us_;
        if(delta>SECOND_MS){
            trackedObjects.erase(iter);
            iter--;
        }
    }
}

// struct object_pair{
// 	int assigned_id=0;
// 	int network_id=0;
// 	const artery::VehicleDataProvider* objDataProvider;
// 	Motion_Vector_Global mv;
// };

// struct tracked_object{
// 	object_pair ground_truth;
// 	object_pair lastPred;
// 	FusionUKF fusionUKF;
// };

double convertcctoc(double val){
    return (M_PI/2)-val;
}



void LocalPerceptionNode::dataFusion(){
    std::ofstream myfile;
    myfile.open("results/results_"+runnumber+"/localperception-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
    auto& m_sim = _mVehicleController->getTraCI()->simulation;

    if(trackedObjects.empty()){

        std::map< int, std::vector<object_pair> > newobjectmap = trackedObjectData[current_ts];

        for(auto iter = newobjectmap.begin(); iter!=newobjectmap.end();iter++){
            tracked_object newobj;
            newobj.lastPred.assigned_id=iter->first;
            newobj.lastPred.mv._idlocal=iter->first;

            std::vector<int> idlist;

            for(auto iter2=iter->second.begin();iter2!=iter->second.end();iter2++){
                MeasurementPackage meas_package;

                meas_package.sensor_type_ = MeasurementPackage::LASER;
                meas_package.raw_measurements_ = VectorXd(4);
                // px, py, v, yaw, yawrate
                idlist.push_back(iter2->mv._id);
                auto pos = m_sim.convertGeo(iter2->mv._location._lat._value/(M_PI/180.0)/10000000, iter2->mv._location._lon._value/(M_PI/180.0)/10000000, true);
                meas_package.raw_measurements_ << pos.x, pos.y, iter2->mv._speed._value, convertcctoc(iter2->mv._heading._value);
                meas_package.timestamp_ = current_ts;

                newobj.fusionUKF.ProcessMeasurement(meas_package);

            }

            // // auto& m_sim = _mVehicleController->getTraCI()->simulation;
            auto carPos = m_sim.convertGeo(newobj.fusionUKF.x_(0), newobj.fusionUKF.x_(1), false);
            

            auto val = newobj.lastPred.mv._idlocal;
            newobj.lastPred.mv= iter->second[0].mv.copy_simple();
            newobj.lastPred.mv._idlocal=val;
            
            newobj.lastPred.mv._location._lat._value = round((carPos.x)*boost::units::degree::degree, microdegree5) * Longitude_oneMicrodegreeEast * (M_PI/180.0);
            newobj.lastPred.mv._location._lon._value = round((carPos.y)*boost::units::degree::degree, microdegree5) * Latitude_oneMicrodegreeNorth * (M_PI/180.0);
            newobj.lastPred.mv._speed._value=newobj.fusionUKF.x_(2);
            newobj.lastPred.mv._heading._value=convertcctoc(newobj.fusionUKF.x_(3));
            newobj.lastPred.mv._yaw_rate._value=newobj.fusionUKF.x_(4);
            newobj.lastPred.mv._timestamp=newobj.fusionUKF.time_us_;

            // if(newobj.lastPred.mv._location._lat._value<0 || newobj.lastPred.mv._location._lon._value<0){
            //     myfile<<newobj.lastPred.mv.to_string();
            //     myfile<<newobj.fusionUKF.x_(0)<<" "<< newobj.fusionUKF.x_(1)<<"\n";
            // }


            trackedObjects.push_back(newobj);
            myfile<<newobj.lastPred.mv.to_string();

        }

    }
    else{

        std::map< int, std::vector<object_pair> > newobjectmap = trackedObjectData[current_ts];

        // iterating matched perception data list
        for(auto iter = newobjectmap.begin(); iter!=newobjectmap.end();iter++){

            // iterating tracked objects UKF
            bool matched=false;
            for(auto tracked = trackedObjects.begin(); tracked!=trackedObjects.end(); tracked++){
                if(iter->first==tracked->lastPred.assigned_id){
                    matched=true;

                    // iterating perception data
                    for(auto iter2= iter->second.begin(); iter2!=iter->second.end(); iter2++){
                        MeasurementPackage meas_package;

                        meas_package.sensor_type_ = MeasurementPackage::LASER;
                        meas_package.raw_measurements_ = VectorXd(4);
                        // px, py, v, yaw, yawrate
                        auto pos = m_sim.convertGeo(iter2->mv._location._lat._value/(M_PI/180.0)/10000000, iter2->mv._location._lon._value/(M_PI/180.0)/10000000, true);
                        meas_package.raw_measurements_ << pos.x, pos.y, iter2->mv._speed._value, convertcctoc(iter2->mv._heading._value);
                        meas_package.timestamp_ = current_ts;

                        tracked->fusionUKF.ProcessMeasurement(meas_package);

                    }

                    auto carPos = m_sim.convertGeo(tracked->fusionUKF.x_(0), tracked->fusionUKF.x_(1), false);
                    
                    tracked->lastPred.mv._location._lat._value = round((carPos.x)*boost::units::degree::degree, microdegree5) * Longitude_oneMicrodegreeEast * (M_PI/180.0);
                    tracked->lastPred.mv._location._lon._value = round((carPos.y)*boost::units::degree::degree, microdegree5) * Latitude_oneMicrodegreeNorth * (M_PI/180.0);
                    tracked->lastPred.mv._speed._value=tracked->fusionUKF.x_(2);
                    tracked->lastPred.mv._heading._value=convertcctoc(tracked->fusionUKF.x_(3));
                    tracked->lastPred.mv._yaw_rate._value=tracked->fusionUKF.x_(4);
                    tracked->lastPred.mv._timestamp=tracked->fusionUKF.time_us_;
                    myfile<<tracked->lastPred.mv.to_string();

                }
            }

            if(!matched){
                tracked_object newobj;
                newobj.lastPred.assigned_id=iter->first;
                newobj.lastPred.mv._idlocal=iter->first;

                for(auto iter2=iter->second.begin();iter2!=iter->second.end();iter2++){
                    MeasurementPackage meas_package;

                    meas_package.sensor_type_ = MeasurementPackage::LASER;
                    meas_package.raw_measurements_ = VectorXd(4);
                    // px, py, v, yaw, yawrate
                    auto pos = m_sim.convertGeo(iter2->mv._location._lat._value/(M_PI/180.0)/10000000, iter2->mv._location._lon._value/(M_PI/180.0)/10000000, true);
                    meas_package.raw_measurements_ << pos.x, pos.y, iter2->mv._speed._value, convertcctoc(iter2->mv._heading._value);
                    meas_package.timestamp_ = current_ts;

                    newobj.fusionUKF.ProcessMeasurement(meas_package);

                    
                }

                

                // // auto& m_sim = _mVehicleController->getTraCI()->simulation;
                auto carPos = m_sim.convertGeo(newobj.fusionUKF.x_(0), newobj.fusionUKF.x_(1), false);
                
                auto val = newobj.lastPred.mv._idlocal;
                newobj.lastPred.mv= iter->second[0].mv.copy_simple();
                newobj.lastPred.mv._idlocal=val;
                
                newobj.lastPred.mv._location._lat._value = round((carPos.x)*boost::units::degree::degree, microdegree5) * Longitude_oneMicrodegreeEast * (M_PI/180.0);
                newobj.lastPred.mv._location._lon._value = round((carPos.y)*boost::units::degree::degree, microdegree5) * Latitude_oneMicrodegreeNorth * (M_PI/180.0);
                newobj.lastPred.mv._speed._value=newobj.fusionUKF.x_(2);
                newobj.lastPred.mv._heading._value=convertcctoc(newobj.fusionUKF.x_(3));
                newobj.lastPred.mv._yaw_rate._value=newobj.fusionUKF.x_(4);
                newobj.lastPred.mv._timestamp=newobj.fusionUKF.time_us_;

                // if(newobj.lastPred.mv._location._lat._value<0 || newobj.lastPred.mv._location._lon._value<0){
                //     myfile<<newobj.lastPred.mv.to_string();
                //     myfile<<newobj.fusionUKF.x_(0)<<" "<< newobj.fusionUKF.x_(1)<<"\n";
                // }

                trackedObjects.push_back(newobj);
                myfile<<newobj.lastPred.mv.to_string();
                
            }
        }

    }
    
    myfile.close();
}

void LocalPerceptionNode::objectAssociation(){

    if(trackedObjects.empty()){
        trackedObjectData[current_ts]=untrackedObjectMatch();
    }
    else {
        trackedObjectData[current_ts]=trackedObjectMatch();
    }
}

double LocalPerceptionNode::angleDiff(double ang1, double ang2){
    return 180.0/M_PI - abs( abs(ang1 - ang2) - 180.0/M_PI);
}

double LocalPerceptionNode::singularConfidence(double testval, double prediction, double MAE){
    double diff = abs(testval-prediction);
    double eval = beta*MAE;
    if(diff <= eval){
        return 1;
    }
    double ret = 1 - (diff - eval)/( alpha*MAE);
    if(ret<0) return 0;
    return ret;
}

double LocalPerceptionNode::mvConfidence(Motion_Vector_Global target, UKF verify){
    auto& m_sim = _mVehicleController->getTraCI()->simulation;
    auto pos = m_sim.convertGeo(target._location._lat._value/(M_PI/180.0)/10000000, target._location._lon._value/(M_PI/180.0)/10000000, true);

    double x_conf = singularConfidence(pos.x, verify.x_(0), MAE_posx);
    double y_conf = singularConfidence(pos.y, verify.x_(1), MAE_posy);
    double speed_conf = singularConfidence(target._speed._value, verify.x_(2), MAE_speed);
    
    
    double headingdelta = convertcctoc(target._heading._value)/verify.x_(3);

    headingdelta = angleDiff(convertcctoc(target._heading._value), verify.x_(3));
    headingdelta = 1 - headingdelta;



    // std::cout<<"theading: "<<convertcctoc(target._heading._value)<<" vheading: "<<verify.x_(3)<<"\n";
    // std::cout<<"x: "<<x_conf<<" y: "<<y_conf<<" s: "<<speed_conf<<" h: "<<headingdelta<<" misc: "<< "\n";
    // double dist = calculate_distance(pos.x, pos.y, test.x_(0), test.x_(1));

    // iter2->mv._speed._value, convertcctoc(iter2->mv._heading._value)

    return x_conf*y_conf*speed_conf;
}

std::map< int, std::vector<object_pair> > LocalPerceptionNode::trackedObjectMatch(){

    std::vector<object_pair> currentInputs;
    auto search2 = rawPerceptionData.find(current_ts);
    if (search2 != rawPerceptionData.end()){


        currentInputs=rawPerceptionData[current_ts];
        // tcurrentInputs.begin()->
    }

    auto& m_sim = _mVehicleController->getTraCI()->simulation;

    std::map< int, std::vector<object_pair> > newobjectmap;

    std::ofstream myfile;
    myfile.open("results/results_"+runnumber+"/localperception-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);

    // auto firstPos = rawPerceptionData.rbegin();
    // for(auto iter2=firstPos->second.begin(); iter2!=firstPos->second.end(); iter2++){
    //     myfile<<iter2->mv.to_string();
    // }


    for(auto iter2=trackedObjects.begin(); iter2!=trackedObjects.end(); iter2++){
        double dt = (current_ts-iter2->fusionUKF.time_us_)/ 1000000.0;
        UKF test = iter2->fusionUKF.UKF_copy();
        test.Prediction(dt);

        // myfile<<"match to obj "<<iter2->lastPred.assigned_id<<":\n";

        for(auto iter1=currentInputs.begin(); iter1!=currentInputs.end(); iter1++){
            auto pos = m_sim.convertGeo(iter1->mv._location._lat._value/(M_PI/180.0)/10000000, iter1->mv._location._lon._value/(M_PI/180.0)/10000000, true);
            double dist = calculate_distance(pos.x, pos.y, test.x_(0), test.x_(1));
            // myfile<<iter1->mv.to_string();
            // myfile<<"dist: "<<dist<<"\n";
            if(dist<=DIST_THRESHOLD){

                // Confidence threshold
                double conf = mvConfidence(iter1->mv, test);

                // std::cout<<"conf: "<<conf<<"\n";

                if(conf<0.5){
                    // std::cout<<"skip for low confidence\n";
                    continue;
                }

                // std::cout<<"continue due to  confidence\n";

                // Distance threshold only
                newobjectmap[iter2->lastPred.assigned_id].push_back(*iter1);
                
                // Confidence calculation
                // double headingdelta = angleDiff(convertcctoc(iter1->mv._heading._value), test.x_(3));
                // if(abs(headingdelta)<1.0){
                //     newobjectmap[iter2->lastPred.assigned_id].push_back(*iter1);
                //     // myfile<<"added item\n";
                    
                // }
            }
        }

    }

    for(auto iter1=currentInputs.begin(); iter1!=currentInputs.end(); iter1++){
        if(!insideMap(newobjectmap, iter1)){
            // myfile<<"not inside map\n";
            std::vector<object_pair> newvec;
            newvec.push_back(*iter1);
            newobjectmap[++currID]= newvec;
            for(auto iter2=iter1+1; iter2!=currentInputs.end(); iter2++){
                if(iter1!=iter2){
                    auto pos = m_sim.convertGeo(iter1->mv._location._lat._value/(M_PI/180.0)/10000000, iter1->mv._location._lon._value/(M_PI/180.0)/10000000, true);
                    auto pos2 = m_sim.convertGeo(iter2->mv._location._lat._value/(M_PI/180.0)/10000000, iter2->mv._location._lon._value/(M_PI/180.0)/10000000, true);

                    double dist = calculate_distance(pos.x, pos.y, pos2.x, pos2.y);
                    if(dist<=DIST_THRESHOLD){
                        iter2->assigned_id=currID;
                        iter2->mv._idlocal=currID;
                        newobjectmap[currID].push_back(*iter2);
                        // myfile<<"added item\n";
                    }
                }
            }
        }
        // currID++;
        // myfile<<"\n";
    }
    //         myfile<<"not inside map\n";
    //         std::vector<object_pair> newvec;
    //         newvec.push_back(*iter1);
    //         newobjectmap[currID]= newvec;
    //         for(auto iter2=trackedObjects.begin(); iter2!=trackedObjects.end(); iter2++){

    //             double dt = (current_ts-test.time_us_)/ 1000000.0;

    //             test.Prediction(dt);

    //             // auto carPos = m_sim.convertGeo(test.x_(0), test.x_(1), false);
    //             auto pos = m_sim.convertGeo(iter1->mv._location._lat._value/(M_PI/180.0)/10000000, iter1->mv._location._lon._value/(M_PI/180.0)/10000000, true);
    //             // auto pos2 = m_sim.convertGeo(iter2->mv._location._lat._value/(M_PI/180.0)/10000000, iter2->mv._location._lon._value/(M_PI/180.0)/10000000, true);
    //             double dist = calculate_distance(pos.x, pos.y, test.x_(0), test.x_(1));
    //             if(dist<=DIST_THRESHOLD){
    //                 newobjectmap[currID].push_back(*iter1);
    //                 myfile<<"added item\n";
    //             }
                
    //         }
    //     }
    //     currID++;
    //     // myfile<<"\n";
    // }
    
    // myfile<<"found "<<newobjectmap.size()<<" objects\n";
    // int count=1;
    // for(auto iter1=newobjectmap.begin();iter1!=newobjectmap.end();iter1++){
    //     myfile<<"obj "<<count<<" has "<<iter1->second.size()<<" mvs\n";
    //     count++;
    // }

    // myfile.close();

    // trackedObjectData[current_ts];

    return newobjectmap;

}

std::map< int, std::vector<object_pair> > LocalPerceptionNode::untrackedObjectMatch(){
// void LocalPerceptionNode::untrackedObjectMatch(){

    std::vector<object_pair> currentInputs;
    auto search2 = rawPerceptionData.find(current_ts);
    if (search2 != rawPerceptionData.end()){


        currentInputs=rawPerceptionData[current_ts];
        // tcurrentInputs.begin()->
    }

    auto& m_sim = _mVehicleController->getTraCI()->simulation;

    std::map< int, std::vector<object_pair> > newobjectmap;

    std::ofstream myfile;
    myfile.open("results/results_"+runnumber+"/localperception-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);

    for(auto iter1=currentInputs.begin(); iter1!=currentInputs.end(); iter1++){
        if(!insideMap(newobjectmap, iter1)){
            // myfile<<"not inside map\n";
            std::vector<object_pair> newvec;
            newvec.push_back(*iter1);
            newobjectmap[++currID]= newvec;
            for(auto iter2=iter1+1; iter2!=currentInputs.end(); iter2++){
                if(iter1!=iter2){
                    auto pos = m_sim.convertGeo(iter1->mv._location._lat._value/(M_PI/180.0)/10000000, iter1->mv._location._lon._value/(M_PI/180.0)/10000000, true);
                    auto pos2 = m_sim.convertGeo(iter2->mv._location._lat._value/(M_PI/180.0)/10000000, iter2->mv._location._lon._value/(M_PI/180.0)/10000000, true);

                    double dist = calculate_distance(pos.x, pos.y, pos2.x, pos2.y);
                    if(dist<=DIST_THRESHOLD){
                        iter2->assigned_id=currID;
                        iter2->mv._idlocal=currID;
                        newobjectmap[currID].push_back(*iter2);
                        // myfile<<"added item\n";
                    }
                }
            }
        }
        // currID++;
        // myfile<<"\n";
    }
    
    // myfile<<"found "<<newobjectmap.size()<<" objects\n";
    // int count=1;
    // for(auto iter1=newobjectmap.begin();iter1!=newobjectmap.end();iter1++){
    //     myfile<<"obj "<<count<<" has "<<iter1->second.size()<<" mvs\n";
    //     count++;
    // }

    myfile.close();

    return newobjectmap;
}

bool LocalPerceptionNode::insideMap(std::map< int, std::vector<object_pair> > objmap, std::vector<object_pair>::iterator iter){

    for(auto iter1=objmap.begin();iter1!=objmap.end();iter1++){
        for(auto iter2=iter1->second.begin();iter2!=iter1->second.end();iter2++){
            if(iter->mv.to_string().compare(iter2->mv.to_string())==0){
                // std::cout<<"returned true\n";
                return true;
            }
        }
    }

    return false;
}

bool LocalPerceptionNode::insideGT(object_pair newdata){

    for(auto iter1=groundTruth.begin();iter1!=groundTruth.end();iter1++){
        for(auto iter2=iter1->second.begin();iter2!=iter1->second.end();iter2++){
            // if(newdata.mv._id==iter2->_id && newdata.mv._timestamp==iter2->_timestamp)
            if(newdata.mv.to_string().compare(iter2->to_string())==0)
                return true;
        }
    }

    return false;
}



double LocalPerceptionNode::calculate_distance(const double x1, const double y1, const double x2, const double y2){
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0); 
}

void LocalPerceptionNode::addDataCAM(object_pair newdata){
    // Register ground truth
    // auto search2 = groundTruth.find(newdata.mv._timestamp);
    // if (search2 != groundTruth.end()){
    //     if(!insideGT(newdata)){
    //         groundTruth[newdata.mv._timestamp].push_back(newdata.mv);
    //     }
    // }
    // else{
    //     std::vector<Motion_Vector_Global> newvec;
    //     newvec.push_back(newdata.mv);
    //     groundTruth[newdata.mv._timestamp]=newvec;
    // }

    auto thists=_mVehicleDataProvider->updated().inUnit(omnetpp::SIMTIME_US);

    // if(newdata.mv._timestamp != thists){
    std::ofstream myfile;
    myfile.open("results/results_"+runnumber+"/oldcam-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
    myfile<<thists<<" "<<newdata.mv._timestamp<<"\n";
    myfile.close();
    // }

    Motion_Vector_Global gt = newdata.mv.copy_simple();

    // rawPerceptionData input
    auto& m_sim = _mVehicleController->getTraCI()->simulation;
    auto carPos = m_sim.convertGeo(_mVehicleDataProvider->latitude().value(), _mVehicleDataProvider->longitude().value(), true);

    auto objpos = m_sim.convertGeo(newdata.mv._location._lat._value/(M_PI/180.0)/10000000, newdata.mv._location._lon._value/(M_PI/180.0)/10000000, true);

    double distance = 1;
    typedef boost::numeric::converter<artery::GeoPosition::value_type, double> Double2Geopos;

    auto search = rawPerceptionData.find(newdata.mv._timestamp);
    if (search != rawPerceptionData.end()){
        Motion_Vector_Global newmv = newdata.mv.copy_with_noise(distance, m_sim);
        newmv._location._lat._value = round((newmv._location._lat._value)*boost::units::degree::degree, microdegree5) * Longitude_oneMicrodegreeEast * (M_PI/180.0);
        newmv._location._lon._value = round((newmv._location._lon._value)*boost::units::degree::degree, microdegree5) * Latitude_oneMicrodegreeNorth * (M_PI/180.0);
        // std::ofstream myfile;
        // myfile.open("results/localperception-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
        // myfile<<"carPos1: "<<carPos.x<<" "<<carPos.y<<" pos2: "<<objpos.x<<" "<<objpos.y<<"\n";
        // myfile<<"distance: "<<distance<<" "<<((int)distance / 5) <<"\n";
        // myfile<<newdata.mv.to_string();
        // myfile<<newmv.to_string();
        // myfile.close();
        newdata.mv=newmv;
        rawPerceptionData[newdata.mv._timestamp].push_back(newdata);
    }
    else{
        std::vector<object_pair> newvec;
        Motion_Vector_Global newmv = newdata.mv.copy_with_noise(distance, m_sim);
        newmv._location._lat._value = round((newmv._location._lat._value)*boost::units::degree::degree, microdegree5) * Longitude_oneMicrodegreeEast * (M_PI/180.0);
        newmv._location._lon._value = round((newmv._location._lon._value)*boost::units::degree::degree, microdegree5) * Latitude_oneMicrodegreeNorth * (M_PI/180.0);
        // std::ofstream myfile;
        // myfile.open("results/localperception-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
        // myfile<<"carPos1: "<<carPos.x<<" "<<carPos.y<<" pos2: "<<objpos.x<<" "<<objpos.y<<"\n";
        // myfile<<"distance: "<<distance<<" "<<((int)distance / 5) <<"\n";
        // myfile<<newdata.mv.to_string();
        // myfile<<newmv.to_string();
        // myfile.close();
        newdata.mv=newmv;
        newvec.push_back(newdata);
        rawPerceptionData[newdata.mv._timestamp]=newvec;
    }

    //std::ofstream myfile;
    myfile.open("results/results_"+runnumber+"/uniqueids.out",  std::ios_base::app);
    myfile << newdata.mv._id<<"\n";
    myfile.close();


    // std::ofstream myfile;
    // myfile.open("results/datafusion-"+std::to_string(gt._id)+".out",  std::ios_base::app);
    
    // objpos = m_sim.convertGeo(newdata.mv._location._lat._value/(M_PI/180.0)/10000000, newdata.mv._location._lon._value/(M_PI/180.0)/10000000, true);
    // auto gtpos = m_sim.convertGeo(gt._location._lat._value/(M_PI/180.0)/10000000, gt._location._lon._value/(M_PI/180.0)/10000000, true);
    // myfile<<"L\t"<<objpos.x<<"\t"<<objpos.y<<"\t"<<newdata.mv._speed._value<<"\t"<<newdata.mv._heading._value<<"\t"<<newdata.mv._timestamp<<"\t"<< gtpos.x<<"\t"<<gtpos.y<<"\t"<<std::to_string(sin(gt._heading._value)*gt._speed._value)<<"\t"<<std::to_string(cos(gt._heading._value)*gt._speed._value)<<"\n";
    
    // myfile.close();
}

void LocalPerceptionNode::addDataCPM(object_pair newdata, struct libsumo::TraCIPosition carPos){
    // std::ofstream myfile;
    // myfile.open("results/localperception-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
    // // myfile<<"carPos1: "<<carPos.x<<" "<<carPos.y<<" pos2: "<<objpos.x<<" "<<objpos.y<<"\n";
    // // myfile<<"distance: "<<distance<<" "<<((int)distance / 5) <<"\n";
    // myfile<<"adding: "<<newdata.mv.to_string();
    // // myfile<<newmv.to_string();
    // myfile.close();

    // Register ground truth
    // auto search2 = groundTruth.find(newdata.mv._timestamp);
    // if (search2 != groundTruth.end()){
    //     if(!insideGT(newdata)){
    //         groundTruth[newdata.mv._timestamp].push_back(newdata.mv);
    //     }
    // }
    // else{
    //     std::vector<Motion_Vector_Global> newvec;
    //     newvec.push_back(newdata.mv);
    //     groundTruth[newdata.mv._timestamp]=newvec;
    // }

    

    // rawPerceptionData input
    auto& m_sim = _mVehicleController->getTraCI()->simulation;
    // auto carPos = m_sim.convertGeo(origDataProvider->latitude().value(), origDataProvider->longitude().value(), true);

    auto objpos = m_sim.convertGeo(newdata.mv._location._lat._value/(M_PI/180.0)/10000000, newdata.mv._location._lon._value/(M_PI/180.0)/10000000, true);

    double distance = calculate_distance(carPos.x, carPos.y, objpos.x, objpos.y);
    typedef boost::numeric::converter<artery::GeoPosition::value_type, double> Double2Geopos;

    auto search = rawPerceptionData.find(newdata.mv._timestamp);
    if (search != rawPerceptionData.end()){
        Motion_Vector_Global newmv = newdata.mv.copy_with_noise(distance, m_sim);
        newmv._location._lat._value = round((newmv._location._lat._value)*boost::units::degree::degree, microdegree5) * Longitude_oneMicrodegreeEast * (M_PI/180.0);
        newmv._location._lon._value = round((newmv._location._lon._value)*boost::units::degree::degree, microdegree5) * Latitude_oneMicrodegreeNorth * (M_PI/180.0);
        // std::ofstream myfile;
        // myfile.open("results/localperception-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
        // myfile<<"carPos1: "<<carPos.x<<" "<<carPos.y<<" pos2: "<<objpos.x<<" "<<objpos.y<<"\n";
        // myfile<<"distance: "<<distance<<" "<<((int)distance / 5) <<"\n";
        // myfile<<newdata.mv.to_string();
        // myfile<<newmv.to_string();
        // myfile.close();
        newdata.mv=newmv;
        rawPerceptionData[newdata.mv._timestamp].push_back(newdata);
    }
    else{
        std::vector<object_pair> newvec;
        Motion_Vector_Global newmv = newdata.mv.copy_with_noise(distance, m_sim);
        newmv._location._lat._value = round((newmv._location._lat._value)*boost::units::degree::degree, microdegree5) * Longitude_oneMicrodegreeEast * (M_PI/180.0);
        newmv._location._lon._value = round((newmv._location._lon._value)*boost::units::degree::degree, microdegree5) * Latitude_oneMicrodegreeNorth * (M_PI/180.0);
        // std::ofstream myfile;
        // myfile.open("results/localperception-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
        // myfile<<"carPos1: "<<carPos.x<<" "<<carPos.y<<" pos2: "<<objpos.x<<" "<<objpos.y<<"\n";
        // myfile<<"distance: "<<distance<<" "<<((int)distance / 5) <<"\n";
        // myfile<<newdata.mv.to_string();
        // myfile<<newmv.to_string();
        // myfile.close();
        newdata.mv=newmv;
        newvec.push_back(newdata);
        rawPerceptionData[newdata.mv._timestamp]=newvec;
    }

    std::ofstream myfile;
    myfile.open("results/results_"+runnumber+"/uniqueids-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
    myfile << newdata.mv._id<<"\n";
    myfile.close();
}



void LocalPerceptionNode::addDataLocal(object_pair newdata){
    // Register ground truth
    // auto search2 = groundTruth.find(newdata.mv._timestamp);
    // if (search2 != groundTruth.end()){
    //     if(!insideGT(newdata)){
    //         groundTruth[newdata.mv._timestamp].push_back(newdata.mv);
    //     }
    // }
    // else{
    //     std::vector<Motion_Vector_Global> newvec;
    //     newvec.push_back(newdata.mv);
    //     groundTruth[newdata.mv._timestamp]=newvec;
    // }

    // rawPerceptionData input
    auto& m_sim = _mVehicleController->getTraCI()->simulation;
    auto carPos = m_sim.convertGeo(_mVehicleDataProvider->latitude().value(), _mVehicleDataProvider->longitude().value(), true);

    auto objpos = m_sim.convertGeo(newdata.mv._location._lat._value/(M_PI/180.0)/10000000, newdata.mv._location._lon._value/(M_PI/180.0)/10000000, true);

    double distance = calculate_distance(carPos.x, carPos.y, objpos.x, objpos.y);
    typedef boost::numeric::converter<artery::GeoPosition::value_type, double> Double2Geopos;

    auto search = rawPerceptionData.find(newdata.mv._timestamp);
    if (search != rawPerceptionData.end()){
        Motion_Vector_Global newmv = newdata.mv.copy_with_noise(distance, m_sim);
        newmv._location._lat._value = round((newmv._location._lat._value)*boost::units::degree::degree, microdegree5) * Longitude_oneMicrodegreeEast * (M_PI/180.0);
        newmv._location._lon._value = round((newmv._location._lon._value)*boost::units::degree::degree, microdegree5) * Latitude_oneMicrodegreeNorth * (M_PI/180.0);
        // std::ofstream myfile;
        // myfile.open("results/localperception-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
        // myfile<<"carPos1: "<<carPos.x<<" "<<carPos.y<<" pos2: "<<objpos.x<<" "<<objpos.y<<"\n";
        // myfile<<"distance: "<<distance<<" "<<((int)distance / 5) <<"\n";
        // myfile<<newdata.mv.to_string();
        // myfile<<newmv.to_string();
        // myfile.close();
        newdata.mv=newmv;
        rawPerceptionData[newdata.mv._timestamp].push_back(newdata);
    }
    else{
        std::vector<object_pair> newvec;
        Motion_Vector_Global newmv = newdata.mv.copy_with_noise(distance, m_sim);
        newmv._location._lat._value = round((newmv._location._lat._value)*boost::units::degree::degree, microdegree5) * Longitude_oneMicrodegreeEast * (M_PI/180.0);
        newmv._location._lon._value = round((newmv._location._lon._value)*boost::units::degree::degree, microdegree5) * Latitude_oneMicrodegreeNorth * (M_PI/180.0);
        // std::ofstream myfile;
        // myfile.open("results/localperception-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
        // myfile<<"carPos1: "<<carPos.x<<" "<<carPos.y<<" pos2: "<<objpos.x<<" "<<objpos.y<<"\n";
        // myfile<<"distance: "<<distance<<" "<<((int)distance / 5) <<"\n";
        // myfile<<newdata.mv.to_string();
        // myfile<<newmv.to_string();
        // myfile.close();
        newdata.mv=newmv;
        newvec.push_back(newdata);
        rawPerceptionData[newdata.mv._timestamp]=newvec;
    }
    std::ofstream myfile;
    myfile.open("results/results_"+runnumber+"/uniqueids-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
    myfile << newdata.mv._id<<"\n";
    myfile.close();

}

void LocalPerceptionNode::registerOldest(std::map< Time, std::vector<object_pair> > data){
    auto firstPos = data.begin();
    std::ofstream myfile;
    myfile.open("results/results_"+runnumber+"/localperceptionGT-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
    for(auto iter2=firstPos->second.begin(); iter2!=firstPos->second.end(); iter2++){
        myfile<<iter2->mv.to_string();
    }
    myfile.close();
}

void LocalPerceptionNode::registerOldestGT(){
    auto tstoremove=current_ts-100000*9;
    auto search2 = groundTruth.find(tstoremove);
    auto toremove = groundTruth[current_ts];
    bool found=false;
    if (search2 != groundTruth.end()){
        toremove=groundTruth[tstoremove];
        found=true;
    }
    if(!found) return;
    std::ofstream myfile;
    myfile.open("results/results_"+runnumber+"/localperceptionGT-"+_mVehicleController->getVehicleId()+".out",  std::ios_base::app);
    for(auto iter2=toremove.begin(); iter2!=toremove.end(); iter2++){
        myfile<<iter2->to_string();
    }
    myfile.close();

    while(toremove.size()>0){
        toremove.erase(toremove.begin());
    }
    groundTruth.erase(search2);
}

void LocalPerceptionNode::removeOldest(std::map< Time, std::vector<object_pair> > data){
    auto tstoremove=current_ts-100000*9;
    auto search2 = data.find(tstoremove);
    auto toremove = data[current_ts];
    bool found=false;
    if (search2 != data.end()){
        toremove=data[tstoremove];
        found=true;
    }
    if(!found) return;
    std::ofstream myfile;
    while(toremove.size()>0){
        toremove.erase(toremove.begin());
    }
    data.erase(search2);
}

void LocalPerceptionNode::removeOldest(std::map< Time, std::map< int, std::vector<object_pair> > > data){
    auto tstoremove=current_ts-100000*9;
    auto search2 = data.find(tstoremove);
    auto toremove = data[current_ts];
    bool found=false;
    if (search2 != data.end()){
        toremove=data[tstoremove];
        found=true;
    }
    if(!found) return;

    while(toremove.size()>0){
        auto secondPos = toremove.begin();

        while(secondPos->second.size()>0){
            secondPos->second.erase(secondPos->second.begin());
        }

        toremove.erase(secondPos);
    }
    data.erase(search2);
}

std::string LocalPerceptionNode::data_to_string(){
    std::stringstream sts;

    for(auto iter=rawPerceptionData.begin(); iter!=rawPerceptionData.end();iter++){
        sts<<"timestamp: "<<iter->first<<"\n";

        for(auto iter2=iter->second.begin(); iter2!=iter->second.end(); iter2++){
            sts<<iter2->mv.to_string();
        }
    }

    std::string s = sts.str();
    return s;
}