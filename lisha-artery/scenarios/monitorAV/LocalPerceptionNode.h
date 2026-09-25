#ifndef LocalPerceptionNode_H_
#define LocalPerceptionNode_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/networking/PositionProvider.h"
#include "artery/utility/Geometry.h"
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>
#include "motion_vector.h"
#include "artery/application/VehicleKinematics.h"
// #include "artery/envmod/LocalEnvironmentModel.h"

#include <Eigen/Dense>
#include "ukf/tools.h"
#include "ukf/ukf.h"
#include "ukf/measurement_package.h"

#define INPUT_COUNT 5

// namespace artery
// {

class VehicleDataProvider;
// namespace artery { class LocalEnvironmentModel; }

struct object_pair{
	int assigned_id=0;
	int network_id=0;
	// const artery::VehicleDataProvider* objDataProvider;
	Motion_Vector_Global mv;
};

struct tracked_object{
	object_pair lastPred;
	UKF fusionUKF;
};

class LocalPerceptionNode : public artery::ItsG5BaseService
{
	public:
		virtual ~LocalPerceptionNode();
		void initialize() override;
		void trigger() override;
		void addDataLocal(object_pair newdata);
		void addDataCAM(object_pair newdata);
		void addDataCPM(object_pair newdata, struct libsumo::TraCIPosition origDataProvider);
		std::vector<tracked_object> getTrackedObjects();
		double mvConfidence(Motion_Vector_Global target, UKF verify);
		double angleDiff(double ang1, double ang2);
		double singularConfidence(double testval, double prediction, double MAE);

	private:
		double calculate_distance(const double x1, const double y1, const double x2, const double y2);
		void objectAssociation();
		void dataFusion();
		std::map< int, std::vector<object_pair> > trackedObjectMatch();
		std::map< int, std::vector<object_pair> > untrackedObjectMatch();
		// void untrackedObjectMatch();
		bool insideMap(std::map< int, std::vector<object_pair> > objmap, std::vector<object_pair>::iterator iter);
		std::string data_to_string();
		void registerOldest(std::map< Time, std::vector<object_pair> > data);
		void registerOldestGT();
		bool insideGT(object_pair newdata);
		void removeOldest(std::map< Time, std::vector<object_pair> > data);
		void removeOldest(std::map< Time, std::map< int, std::vector<object_pair> > > data);
		void pruneOldObjects();
		
		std::vector<tracked_object> trackedObjects;
		std::map< Time, std::map< int, std::vector<object_pair> > > trackedObjectData;
		std::map< Time, std::vector<object_pair> > rawPerceptionData;
		std::map< Time, std::vector<Motion_Vector_Global> > groundTruth;
		const artery::Timer* mTimer = nullptr;
        const traci::VehicleController* _mVehicleController = nullptr;
        const artery::VehicleDataProvider* _mVehicleDataProvider = nullptr;
        int mHostId = 0;
		int currID=0;
        const artery::PositionProvider* mPositionProvider = nullptr;
		Time current_ts;
		std::string runnumber;
        // const artery::LocalEnvironmentModel* localEnvmod;
		double alpha = 5;
		double beta = 5;

		double MAE_speed = 0.338670881372713;
		double MAE_heading = 0.5624592714492362;
		double MAE_posx = 0.5730185861132465;
		double MAE_posy = 0.514126914065783;
	
};

// } // namespace artery

#endif /* LocalPerceptionNode_H_ */
