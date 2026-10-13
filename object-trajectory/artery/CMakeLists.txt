# The simulator-coupled half of the object-trajectory experiment: one recorder, nothing else.
#
# A feature of its own rather than a source added to cp_trustfulness, so Paper 1's build is
# untouched and this experiment can be removed in one line. It does link against cp_trustfulness,
# because the oracle is populated by that target's SensorNoiseService and the CPM parsing headers
# live there -- the two reuses the plan grants.
#
# The pure C++17 half (engine, tests, cost evaluator) is NOT built here. It has its own Makefile and
# must stay buildable with nothing but a compiler, because it has to run on the VisionFive 2.

add_artery_feature(object_trajectory
    ArteryDatasetGenerator.cc
)

target_include_directories(object_trajectory PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../cp-trustfulness/include          # Geodesy, CpmTypes
    ${CMAKE_CURRENT_SOURCE_DIR}/../../cp-trustfulness/artery/include   # ArteryAdapters, EvaluationTruth
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# SensorNoiseService (the oracle source) and the parsers' dependencies.
target_link_libraries(object_trajectory PUBLIC cp_trustfulness)

if(WITH_ENVMOD)
    target_link_libraries(object_trajectory PUBLIC envmod)
endif()

# A dedicated run helper. Both NED folders are needed: this one for object_trajectory.*, and
# cp-trustfulness/artery for config.neds.* (HonestCar, SensorNoiseService) and its omnetpp.ini.
# Careful: ../../artery is the Artery FRAMEWORK now that this lives at the repo root, not the
# cp-trustfulness scenario -- every hop into cp-trustfulness names it.
add_opp_run(object_trajectory
    NED_FOLDERS
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../../cp-trustfulness/artery
)
