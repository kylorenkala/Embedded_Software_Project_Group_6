#ifndef COMMON_H
#define COMMON_H

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <iostream>
#include <pthread.h>
#include <stdint.h>

//-------------UDP----------------------------

enum class msgType {
    DISTANCE_SETPOINT,
    DISTANCE_ACTUAL
};

// from leader's perspective
struct txPlatoonMessageFrame {
    msgType messageType = msgType::DISTANCE_SETPOINT;
    unsigned short distance_setpoint;
    bool emergency_brake_leader;
};

struct rxPlatoonMessageFrame {
    msgType messageType = msgType::DISTANCE_ACTUAL;
    unsigned short distance_actual;
    bool emergency_brake_follower;
    int receiver; // 10 - 17
};

//------------Shared Memory-------------------
// from main's perspective
struct rxMainMessageFrame {
    unsigned long position; // calculated in truck and sent to main_frame
    bool request_ready;
};

struct txMainMessageFrame {
    unsigned short sensor_data; // calculated in main_frame and sent to truck every second
    bool response_ready;
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
    int udp_port;
    truck_roles truck_role;
    int position_in_platoon;
    unsigned short distance_report = 0;
};

#endif