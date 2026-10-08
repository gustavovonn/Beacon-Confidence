add_executable(test_cpm_parse test_cpm_parse.cc)
target_include_directories(test_cpm_parse PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../include
    ${OMNETPP_INCLUDE_DIR}
)
target_link_libraries(test_cpm_parse PRIVATE
    envmod
    core
    asn1_its_r2
    asn1
    facilities
    common
    ${OMNETPP_SIM_LIBRARY_RELEASE}
    ${OMNETPP_ENVIR_LIBRARY_RELEASE}
)
add_executable(test_cam_parse test_cam_parse.cc)
target_include_directories(test_cam_parse PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../include
    ${OMNETPP_INCLUDE_DIR}
)
target_link_libraries(test_cam_parse PRIVATE
    asn1_its
    asn1
    facilities
    common
    ${OMNETPP_SIM_LIBRARY_RELEASE}
    ${OMNETPP_ENVIR_LIBRARY_RELEASE}
)
add_executable(test_oracle_path test_oracle_path.cc)
target_include_directories(test_oracle_path PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../include
    ${OMNETPP_INCLUDE_DIR}
)
target_link_libraries(test_oracle_path PRIVATE
    core
    envmod
    ${OMNETPP_SIM_LIBRARY_RELEASE}
    ${OMNETPP_ENVIR_LIBRARY_RELEASE}
)
