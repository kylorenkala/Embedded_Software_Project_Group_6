cmake_minimum_required(VERSION 3.20)
project(TruckPlatoon)
set(CMAKE_CXX_STANDARD 17)

# 1. Find Packages
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)
find_package(OpenCL REQUIRED)  # <--- NEW

# 2. Add Executable (Include GpuHandler.h)
add_executable(truck
        main.cpp
        TruckNode.cpp
        network_module.h
        common.h
        GpuHandler.h
)

# 3. Link Libraries (Add OpenCL)
target_link_libraries(truck PRIVATE Threads::Threads OpenMP::OpenMP_CXX OpenCL::OpenCL)

# 4. CRITICAL: Copy fusion.cl to the build folder
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/fusion.cl ${CMAKE_CURRENT_BINARY_DIR}/fusion.cl COPYONLY)

# 5. MinGW Fixes
if(OpenMP_CXX_FOUND)
    target_compile_options(truck PRIVATE ${OpenMP_CXX_FLAGS})
    target_link_options(truck PRIVATE ${OpenMP_CXX_FLAGS})
endif()

if(WIN32)
    target_link_libraries(truck PRIVATE ws2_32)
endif()