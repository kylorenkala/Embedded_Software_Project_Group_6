#include <fcntl.h>
#include <arpa/inet.h>
#include <cstring>
#include <sys/mman.h>
#include <sys/time.h>
#include <errno.h>
#include <unistd.h>
#include <iostream>
#include <pthread.h>
#include <stdint.h>
#include "common.h"
#include "truck_leader.cpp"
#include "truck_follower.cpp"

int main() {

    // initialize truck ---------------------------------------------------------------------------
    // truck id
    std::cout << "Truck ID (any number): "; std::cin >> self.truck_id; 
    
    // truck role
    char role;
    std::cout << "Leader or Follower (initial): "; std::cin >> role;
    
    if (role == 'l') { // leader
        self.truck_role = truck_roles::LEADER;
        self.position_in_platoon = 0;
    }

    else if (role == 'f') { // follower
        self.truck_role = truck_roles::FOLLOWER;
        
        int position;
        std::cout << "Position in platoon (1 - 7): "; std::cin >> position;
        
        if (position >= 1 && position <= 7) self.position_in_platoon = position;
        else {
            perror("invalid position");
            return 1;
        }
    }

    else {
        perror("invalid input");
        return 1;
    }    
    // --------------------------------------------------------------------------------------------


    // allocate shared memory ---------------------------------------------------------------------
    const char* name = "/main_frame_memory";

    int file_descriptor = shm_open(name, O_RDWR, 0666);
    if (file_descriptor == -1) {
        perror("shm_open");
    }

    data_from_main = (SharedMemoryLayout*) mmap(
        nullptr,
        sizeof(SharedMemoryLayout),
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        file_descriptor,
        0
    );

    if (data_from_main == MAP_FAILED) {
        perror("mmap");
    }
    // --------------------------------------------------------------------------------------------

    std::cout << "Truck ID " << self.truck_id << ": " << std::endl;

    if (self.truck_role == truck_roles::LEADER) runLeader();
    else if (self.truck_role == truck_roles::FOLLOWER) runFollower();

    munmap(data_from_main, sizeof(SharedMemoryLayout));
    close(file_descriptor);

    return 0;
}
