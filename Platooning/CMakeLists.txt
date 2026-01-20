cmake_minimum_required(VERSION 3.20)
project(TruckPlatoon)
set(CMAKE_CXX_STANDARD 17)
find_package(Threads REQUIRED)

add_executable(truck
        main.cpp
        TruckNode.cpp
        # Note: VehiclePhysics and PlatoonController are header-only
        # implementations above, so no .cpp needed in this specific list
        # unless you split them into .cpp files too.
        network_module.h
        common.h
)

target_link_libraries(truck PRIVATE Threads::Threads)
if(WIN32)
    target_link_libraries(truck PRIVATE ws2_32)
endif()