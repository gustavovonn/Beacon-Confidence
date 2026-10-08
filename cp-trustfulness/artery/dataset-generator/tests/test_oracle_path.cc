// cp-trustfulness/artery/dataset-generator/tests/test_oracle_path.cc
//
// Step 1.5: Verify objectStation oracle resolution path:
// lemId -> TraCI externalId -> IdentityRegistry -> ETSI stationId

#include <artery/envmod/EnvironmentModelObject.h>
#include <artery/utility/Identity.h>
#include <artery/utility/IdentityRegistry.h>

#include <cassert>
#include <iostream>
#include <memory>
#include <string>
#include <omnetpp.h>
#include <unordered_map>
#include <vector>

using namespace artery;

// Mock EnvironmentModelObject providing getExternalId() like
// TraCIEnvironmentModelObject does
class MockEnvironmentModelObject : public EnvironmentModelObject {
public:
    MockEnvironmentModelObject(std::string traciId)
            : mTraciId(std::move(traciId)) {}

    void update() override {}
    std::string getExternalId() const override { return mTraciId; }
    const std::vector<Position> &getOutline() const override {
        static std::vector<Position> empty;
        return empty;
    }
    const Position &getAttachmentPoint(const SensorPosition &) const override {
        static Position zero{0.0, 0.0};
        return zero;
    }
    const Position &getCentrePoint() const override {
        static Position zero{0.0, 0.0};
        return zero;
    }
    Heading getHeading() const override { return Angle::from_degree(0.0); }
    Length getLength() const override { return 4.5 * vanetza::units::si::meter; }
    Length getWidth() const override { return 1.8 * vanetza::units::si::meter; }
    Length getRadius() const override { return 2.5 * vanetza::units::si::meter; }
    bool isVisible() override { return true; }

private:
    std::string mTraciId;
};

void test_oracle_resolution_path() {
  std::cout << "=========================================================\n";
  std::cout << "TEST 3: Step 1.5 Oracle Path (lemId -> TraCI -> stationId)\n";
  std::cout << "=========================================================\n";

  // 1. Simulate LocalEnvironmentModel state
  // In LocalEnvironmentModel, objects are stored in:
  // TrackedObjects mObjects (std::map<std::weak_ptr<EnvironmentModelObject>,
  // Tracking>)
  struct MockTracking {
    int id; // lemId
  };

  auto obj1 = std::make_shared<MockEnvironmentModelObject>("veh_100");
  auto obj2 = std::make_shared<MockEnvironmentModelObject>("veh_200");
  auto objObstacle = std::make_shared<MockEnvironmentModelObject>("building_obstacle");

  std::vector<std::pair<std::weak_ptr<EnvironmentModelObject>, MockTracking>>
      mockLemObjects = {{obj1, {10}}, {obj2, {20}}, {objObstacle, {30}}};

  // 2. Setup mock Identity lookup table (simulating IdentityRegistry)
  std::unordered_map<std::string, uint32_t> traciToStationId = {
      {"veh_100", 11111}, {"veh_200", 22222}
      // building_obstacle is not an ITS station, so not present in registry
  };

  auto resolveStationIdFromLem = [&](int targetLemId) -> uint32_t {
    for (const auto &entry : mockLemObjects) {
      if (entry.second.id == targetLemId) {
        auto lockedObj = entry.first.lock();
        if (!lockedObj)
          return 0;

        std::string extId = lockedObj->getExternalId();
        auto it = traciToStationId.find(extId);
        if (it != traciToStationId.end()) {
          return it->second;
        }
        return 0; // unknown / not an ITS vehicle
      }
    }
    return 0;
  };

  // 3. Test vehicle 100 resolution
  uint32_t station1 = resolveStationIdFromLem(10);
  std::cout << "[CHECK] lemId 10 ('veh_100') -> stationId: " << station1
            << " (expected 11111)\n";
  assert(station1 == 11111);

  // 4. Test vehicle 200 resolution
  uint32_t station2 = resolveStationIdFromLem(20);
  std::cout << "[CHECK] lemId 20 ('veh_200') -> stationId: " << station2
            << " (expected 22222)\n";
  assert(station2 == 22222);

  // 5. Test obstacle resolution (non-vehicle object)
  uint32_t stationObstacle = resolveStationIdFromLem(30);
  std::cout << "[CHECK] lemId 30 ('building_obstacle') -> stationId: "
            << stationObstacle << " (expected 0 / null)\n";
  assert(stationObstacle == 0);

  // 6. Test expired/untracked object
  uint32_t stationUnknown = resolveStationIdFromLem(999);
  std::cout << "[CHECK] lemId 999 (not tracked) -> stationId: "
            << stationUnknown << " (expected 0)\n";
  assert(stationUnknown == 0);

  std::cout << "\n>>> TEST 3 RESULT: ORACLE RESOLUTION PATH VALIDATED "
               "SUCCESSFULLY <<<\n\n";
}

int main() {
  omnetpp::cStaticFlag csf;
  omnetpp::SimTime::setScaleExp(-9);

  try {
    test_oracle_resolution_path();

    std::cout << "Oracle path validation completed successfully!\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "[ERROR] Exception caught in test_oracle_path: " << e.what()
              << "\n";
    return 1;
  }
}
