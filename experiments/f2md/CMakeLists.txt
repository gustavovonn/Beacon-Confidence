cmake_minimum_required(VERSION 3.16)
project(f2md_evaluator)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

option(USE_ARROW "Enable Apache Arrow / Parquet ingestion" ON)
option(STATIC_BUILD "Enable static binary linking" OFF)

if(USE_ARROW)
    find_package(Arrow REQUIRED)
    find_package(Parquet REQUIRED)
    add_compile_definitions(USE_ARROW)
endif()

if(STATIC_BUILD)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static")
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/veins/modules/application/f2md
    ${CMAKE_CURRENT_SOURCE_DIR}/veins/base/utils
)

# Common sources (OLS-CTRA and BetaReputation from root src/)
file(GLOB_RECURSE COMMON_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/../src/*.cc")

# F2MD sources (excluding OMNeT++/Veins simulation and network wrappers)
file(GLOB_RECURSE F2MD_SOURCES 
    "${CMAKE_CURRENT_SOURCE_DIR}/veins/modules/application/f2md/mdChecks/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/veins/modules/application/f2md/mdBase/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/veins/modules/application/f2md/mdApplications/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/veins/modules/application/f2md/mdSupport/*.cc"
)

# Filter out OMNeT++ / Veins GUI / Network wrappers
list(FILTER F2MD_SOURCES EXCLUDE REGEX ".*HTTPRequest.*")
list(FILTER F2MD_SOURCES EXCLUDE REGEX ".*LocalAttackServer.*")
list(FILTER F2MD_SOURCES EXCLUDE REGEX ".*BsmPrintable.*")
list(FILTER F2MD_SOURCES EXCLUDE REGEX ".*VeReMiPrintable.*")
list(FILTER F2MD_SOURCES EXCLUDE REGEX ".*XmlWriter.*")
list(FILTER F2MD_SOURCES EXCLUDE REGEX ".*JsonWriter.*")
list(FILTER F2MD_SOURCES EXCLUDE REGEX ".*MachineLearningApp.*")
list(FILTER F2MD_SOURCES EXCLUDE REGEX ".*GaussianRandom.*")
list(FILTER F2MD_SOURCES EXCLUDE REGEX ".*RelativeOffset.*")
list(FILTER F2MD_SOURCES EXCLUDE REGEX ".*VarThrePrintable.*")

# Build target: offline_evaluator
add_executable(offline_evaluator 
    offline_evaluator.cc 
    ${COMMON_SOURCES} 
    ${F2MD_SOURCES} 
    veins/modules/application/f2md/mdReport/ProtocolEnforcer.cc
)

if(USE_ARROW)
    target_link_libraries(offline_evaluator PRIVATE arrow_shared parquet_shared)
endif()
