cmake_minimum_required(VERSION 4.4)
project(cpTrustfullnessVeremiExtension)

add_executable(veremi_runner
    veremi_runner.cc
    ../src/TrustEvaluators.cc
    ../src/DeadReckoningModels.cc
)

target_include_directories(veremi_runner PRIVATE ../include)
target_link_libraries(veremi_runner PRIVATE arrow parquet)
