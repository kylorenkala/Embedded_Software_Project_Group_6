cmake_minimum_required(VERSION 3.20)
project(TruckPlatoon)
set(CMAKE_CXX_STANDARD 17)

# 1. Find the Packages
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)

add_executable(truck
        main.cpp
        TruckNode.cpp
        network_module.h
        common.h
)

# 2. Link Libraries (Standard Way)
target_link_libraries(truck PRIVATE Threads::Threads OpenMP::OpenMP_CXX)

# 3. --- THE FIX: FORCE LINKER FLAGS FOR MINGW ---
# This explicitly adds "-fopenmp" to the linker command line.
if(OpenMP_CXX_FOUND)
    target_compile_options(truck PRIVATE ${OpenMP_CXX_FLAGS})
    target_link_options(truck PRIVATE ${OpenMP_CXX_FLAGS})
endif()

# 4. Windows Networking
if(WIN32)
    target_link_libraries(truck PRIVATE ws2_32)
endif()