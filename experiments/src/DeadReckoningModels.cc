#include "../include/DeadReckoningModels.h"

std::unique_ptr<IDeadReckoningModel> DeadReckoningModelFactory::create(
    const std::string& name,
    const std::string& coordSystem)
{
    const bool cartesian = (coordSystem == "cartesian" || coordSystem == "Cartesian");

    if (name == "HistoryAwareCTRA" || name == "historyawarectra" || name == "ols-ctra") {
        if (cartesian) {
            return std::make_unique<CartesianHistoryAwareCtraModel>();
        }
        return std::make_unique<GeographicHistoryAwareCtraModel>();
    }

    if (cartesian) {
        return std::make_unique<CartesianCtraModel>();
    }
    return std::make_unique<GeographicCtraModel>();
}
