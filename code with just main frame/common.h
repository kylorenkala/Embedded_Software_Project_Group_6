#ifndef COMMON_H
#define COMMON_H

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <iostream>
#include <pthread.h>
#include <stdint.h>

//-------------UDP---------------------------
// from leader's perspective
struct txPlatoonMessageFrame {
    unsigned short distance_setpoint;
    bool emergency_brake_leader;
};

struct rxPlatoonMessageFrame {
    unsigned short distance_actual;
    bool emergency_brake_follower;
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

#endif