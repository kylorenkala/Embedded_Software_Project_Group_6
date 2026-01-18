cmake_minimum_required(VERSION 3.20)
project(TruckPlatoon)

set(CMAKE_CXX_STANDARD 17)

# Required for threading (pthreads) and parallel tasks
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)

# --- APP: The Truck Logic ---
add_executable(truck truck.cpp network_module.h common.h)

# Link Threading and OpenMP libraries
target_link_libraries(truck PRIVATE Threads::Threads OpenMP::OpenMP_CXX)

# Link Winsock (Required for Sockets on Windows)
if(WIN32)
    target_link_libraries(truck PRIVATE ws2_32)
endif()