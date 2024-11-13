cmake_minimum_required(VERSION 3.13)

set(TARGET_NAME LOG)
add_library(${TARGET_NAME})

# Find all .c sources in project
file(GLOB_RECURSE glob_sources "*.c")
target_sources(${TARGET_NAME} PRIVATE ${glob_sources})

# Find directories for '#include'
target_include_directories(${TARGET_NAME} PUBLIC ${CMAKE_SOURCE_DIR})