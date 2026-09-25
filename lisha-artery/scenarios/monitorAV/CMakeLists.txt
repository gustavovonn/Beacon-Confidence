add_artery_feature(monitor MonitorMV.cc MonitorCa.cc EgoMotionController.cc LocalPerceptionNode.cc MonitorLocalPerception.cc MonitorCollectivePerception.cc ukf/ukf.cpp ukf/tools.cpp motion_vector.h CollectivePerceptionService.cc CpObject.cc sdDynamicState.cc)
add_opp_run(monitorAV CONFIG omnetpp.ini NED_FOLDERS ${CMAKE_CURRENT_SOURCE_DIR})
SET(RUN_FLAGS "-uCmdenv")
add_opp_run(monitorAVcmd CONFIG omnetpp_SD_framework.ini NED_FOLDERS ${CMAKE_CURRENT_SOURCE_DIR})


# target_sources(monitor PUBLIC monitorCA.cc)
if(WITH_ENVMOD)
    target_sources(monitor PUBLIC MonitorMV.cc MonitorCa.cc LocalPerceptionNode.cc MonitorLocalPerception.cc MonitorCollectivePerception.cc CollectivePerceptionService.cc CpObject.cc)
    target_link_libraries(monitor PUBLIC envmod curl)
endif()
