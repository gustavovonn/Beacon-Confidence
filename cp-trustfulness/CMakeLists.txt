add_artery_feature(cp_trustfulness
    src/algorithms/DeadReckoningModels.cc
    src/algorithms/TrustEvaluators.cc
    src/algorithms/TrustService.cc
    src/algorithms/FaultyCaService.cc
)

if(WITH_ENVMOD)
    target_link_libraries(cp_trustfulness PUBLIC envmod)
endif()

# Configure opp_run helper for this scenario
add_opp_run(cp_trustfulness NED_FOLDERS ${CMAKE_CURRENT_SOURCE_DIR})
