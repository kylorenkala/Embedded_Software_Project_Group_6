#ifndef COMMON_H
#define COMMON_H

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <iostream>
#include <pthread.h>
#include <stdint.h>

int setpoint = 20; // initialize
int truck_speed = 20; // m/s
int distance_travelled;

Platoon self{}; // currently running truck
Platoon platoon[7]; // make a list of trucks in the platoon, populated through UDP

//-------------UDP----------------------------
enum class msg_type : uint16_t {
    LEADER_TO_FOLLOWER = 1,
    FOLLOWER_TO_LEADER = 2
};

// from leader's perspective
struct leaderPlatoonMessageFrame {
    uint16_t type;
    int truck_id;
    unsigned short distance_setpoint;
    bool emergency_brake_leader;
};

struct followerPlatoonMessageFrame {
    uint16_t type;
    int truck_id;
    int position_in_platoon; // 1 - 7
    unsigned short distance_actual;
    bool emergency_brake_follower;
};

//------------Shared Memory-------------------
SharedMemoryLayout* data_from_main = nullptr;

// from main's perspective
struct rxMainMessageFrame {
    unsigned long position; // calculated in truck and sent to main_frame
    bool position_ready;
};

struct txMainMessageFrame {
    unsigned short sensor_data; // calculated in main_frame and sent to truck every second
    bool sensor_data_ready;
};

// read and write (send and receive) for trucks (position and sensor data)
struct SharedMemoryLayout {
    pthread_mutex_t global_mutex;
    rxMainMessageFrame rx_slots[8];
    txMainMessageFrame tx_slots[8];
};

// truck identifiers
enum class truck_roles {
    LEADER,
    FOLLOWER,
    FREE
};

struct Platoon {
    int truck_id;
    truck_roles truck_role;
    int position_in_platoon;
    unsigned short distance_to_front;
    int clock_matrix[8][8];
};

#endif