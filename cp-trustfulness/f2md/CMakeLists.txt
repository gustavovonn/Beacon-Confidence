cmake_minimum_required(VERSION 3.14)
project(F2MD_Dataset_Scorer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find Apache Arrow and Parquet
find_package(Arrow REQUIRED)
find_package(Parquet REQUIRED)

# 1. This allows F2MD's internal #include <veins/modules/application/f2md/...> to work
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 2. This allows your runner to easily #include "F2MDParameters.h" 
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/veins/modules/application/f2md)

# Grab all F2MD core logic files from the new path
file(GLOB_RECURSE F2MD_SOURCES 
    "veins/modules/application/f2md/mdChecks/*.cc"
    "veins/modules/application/f2md/mdBase/*.cc"
    "veins/modules/application/f2md/mdApplications/*.cc"
    "veins/modules/application/f2md/mdSupport/*.cc"
)

# Helper variable for the exclusion list
set(F2MD_BASE "${CMAKE_CURRENT_SOURCE_DIR}/veins/modules/application/f2md")

# Remove files with OMNeT++ or external networking dependencies
list(REMOVE_ITEM F2MD_SOURCES "${F2MD_BASE}/mdSupport/HTTPRequest.cc")
list(REMOVE_ITEM F2MD_SOURCES "${F2MD_BASE}/mdSupport/LocalAttackServer.cc")
list(REMOVE_ITEM F2MD_SOURCES "${F2MD_BASE}/mdSupport/BsmPrintable.cc")
list(REMOVE_ITEM F2MD_SOURCES "${F2MD_BASE}/mdSupport/VeReMiPrintable.cc")
list(REMOVE_ITEM F2MD_SOURCES "${F2MD_BASE}/mdSupport/XmlWriter.cc")
list(REMOVE_ITEM F2MD_SOURCES "${F2MD_BASE}/mdSupport/JsonWriter.cc")
list(REMOVE_ITEM F2MD_SOURCES "${F2MD_BASE}/mdApplications/MachineLearningApp.cc")

# Define the executable with your runner file
add_executable(f2md_runner 
    f2md_runner.cc 
    veins/modules/application/f2md/mdReport/ProtocolEnforcer.cc 
    ${F2MD_SOURCES}
)

# Link Arrow, Parquet, and Math
target_link_libraries(f2md_runner PRIVATE 
    arrow_shared
    parquet_shared
    m
)
