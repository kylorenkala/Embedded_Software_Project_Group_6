#include <iostream>
#include <pthread.h>
#include "TruckNode.h"
#include <csignal>
#include "profiler.h"

int truckIdGlobal; 

void onShutdown(int sig) {
    Profiler::dumpResults(truckIdGlobal);
    exit(0);
}

int main() {
    signal(SIGINT, onShutdown); // detect ctrl c

    int id;
    std::cout << "--- COMPONENT-BASED TRUCK ---\n";
    std::cout << "Enter Truck ID: ";
    std::cin >> id;
    
    truckIdGlobal = id;

    TruckNode truck(id);

    if (id == 0) {
        int size;
        std::cout << "Enter Target Platoon Size: ";
        std::cin >> size;
        truck.setTargetPlatoonSize(size);
    }

    pthread_t t1, t2;
    pthread_create(&t1, nullptr, TruckNode::startComms, &truck); // task 1
    pthread_create(&t2, nullptr, TruckNode::startInput, &truck); // task 2

    truck.runLogic(); // main thread runs logic, task 3

    return 0;
}
