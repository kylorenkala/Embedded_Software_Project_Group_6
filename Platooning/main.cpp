#include <iostream>
#include <pthread.h>
#include "TruckNode.h"

int main() {
    int id;
    std::cout << "--- COMPONENT-BASED TRUCK ---\n";
    std::cout << "Enter Truck ID: ";
    std::cin >> id;

    TruckNode truck(id);

    if (id == 0) {
        int size;
        std::cout << "Enter Target Platoon Size: ";
        std::cin >> size;
        truck.setTargetPlatoonSize(size);
    }

    pthread_t t1, t2;
    pthread_create(&t1, nullptr, TruckNode::startComms, &truck);
    pthread_create(&t2, nullptr, TruckNode::startInput, &truck);

    truck.runLogic(); // Main thread runs logic

    return 0;
}