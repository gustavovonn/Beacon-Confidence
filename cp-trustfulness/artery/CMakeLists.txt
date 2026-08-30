add_artery_feature(cp_trustfulness
    ../src/DeadReckoningModels.cc
    ../src/TrustEvaluators.cc
    src/TrustService.cc
    src/FaultyCaService.cc
)

target_include_directories(cp_trustfulness PUBLIC ../include include)

if(WITH_ENVMOD)
    target_link_libraries(cp_trustfulness PUBLIC envmod)
endif()

# Configure opp_run helper for this scenario
add_opp_run(cp_trustfulness NED_FOLDERS ${CMAKE_CURRENT_SOURCE_DIR})
