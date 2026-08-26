#ifndef DEAD_RECKONING_MODELS_H_
#define DEAD_RECKONING_MODELS_H_

#include "IDeadReckoningModel.h"
#include <memory>
#include <string>

namespace artery
{

class CvDeadReckoningModel : public IDeadReckoningModel
{
public:
    void predict(VehicleState& state, double dt) override;
    double computeError(const VehicleState& state, double actualLat, double actualLon,
                        double actualSpeed, double actualHeading) override;
    std::string getName() const override { return "CV"; }
};

class CaDeadReckoningModel : public IDeadReckoningModel
{
public:
    void predict(VehicleState& state, double dt) override;
    double computeError(const VehicleState& state, double actualLat, double actualLon,
                        double actualSpeed, double actualHeading) override;
    std::string getName() const override { return "CA"; }
};

class CtraDeadReckoningModel : public IDeadReckoningModel
{
public:
    void predict(VehicleState& state, double dt) override;
    double computeError(const VehicleState& state, double actualLat, double actualLon,
                        double actualSpeed, double actualHeading) override;
    std::string getName() const override { return "CTRA"; }
};

class EkfDeadReckoningModel : public IDeadReckoningModel
{
public:
    void predict(VehicleState& state, double dt) override;
    double computeError(const VehicleState& state, double actualLat, double actualLon,
                        double actualSpeed, double actualHeading) override;
    std::string getName() const override { return "EKF"; }
};

class HistoryAwareCtraModel : public IDeadReckoningModel
{
public:
    void predict(VehicleState& state, double dt) override;
    double computeError(const VehicleState& state, double actualLat, double actualLon,
                        double actualSpeed, double actualHeading) override;
    std::string getName() const override { return "HistoryAwareCTRA"; }
private:
    double unwrapHeading(double currentHeading, double previousHeading);
};

class DeadReckoningModelFactory
{
public:
    static std::unique_ptr<IDeadReckoningModel> create(const std::string& name);
};

} // namespace artery

#endif // DEAD_RECKONING_MODELS_H_
