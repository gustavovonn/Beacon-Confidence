cmake_minimum_required(VERSION 3.16)
project(detection_eval)

# Shared V2X motion primitives (header-only)
set(V2X_MISC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libs/v2x-misc)

# This paper's confidence mechanism, shared by both experiments
set(MECHANISM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
file(GLOB MECHANISM_SOURCES "${MECHANISM_DIR}/src/*.cc")

add_executable(veremi_runner
    runner.cc
    ${MECHANISM_SOURCES}
)

target_include_directories(veremi_runner PRIVATE
    ${V2X_MISC_DIR}/include
    ${MECHANISM_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(veremi_runner PRIVATE arrow parquet)
