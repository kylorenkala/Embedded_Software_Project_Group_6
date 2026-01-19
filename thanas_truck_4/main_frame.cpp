#include <iostream>
#include <fcntl.h> 
#include <unordered_map>
#include <vector>
#include <string>
#include <mqueue.h>
#include <unistd.h>
#include "common.h"

// ---------- helpers ----------
std::string hbQueue(int id) {
    return "/mq_hb_" + std::to_string(id);
}

std::string sensorQueue(int id) {
    return "/mq_sensor_" + std::to_string(id);
}

struct WorldTruck {
    double position;
};

int main() {
    std::unordered_map<int, WorldTruck> trucks;
    uint64_t tick = 0;

    std::cout << "MainFrame running\n";
    std::cout << "Type truck ID + Enter to register\n";

    while (true) {
        // ---- manual registration (stable baseline) ----
        if (std::cin.rdbuf()->in_avail()) {
            int id;
            std::cin >> id;

            trucks[id] = {0.0};

            mq_attr attr{};
            attr.mq_maxmsg = 10;

            attr.mq_msgsize = sizeof(Heartbeat);
            mq_open(hbQueue(id).c_str(),
                    O_CREAT | O_WRONLY | O_NONBLOCK,
                    0666, &attr);

            attr.mq_msgsize = sizeof(SensorMsg);
            mq_open(sensorQueue(id).c_str(),
                    O_CREAT | O_WRONLY | O_NONBLOCK,
                    0666, &attr);

            std::cout << "Registered truck " << id << "\n";
        }

        // ---- simple ordered spacing ----
        std::vector<int> order;
        for (auto& p : trucks)
            order.push_back(p.first);

        for (size_t i = 0; i < order.size(); ++i)
            trucks[order[i]].position = i * 25.0;

        // ---- send sensors ----
        for (size_t i = 1; i < order.size(); ++i) {
            SensorMsg s{};
            s.distanceToFront =
                trucks[order[i - 1]].position -
                trucks[order[i]].position;

            mqd_t mq = mq_open(sensorQueue(order[i]).c_str(),
                               O_WRONLY | O_NONBLOCK);
            if (mq != -1) {
                mq_send(mq, (char*)&s, sizeof(s), 0);
                mq_close(mq);
            }
        }

        // ---- heartbeat ----
        for (auto& p : trucks) {
            Heartbeat hb{tick};
            mqd_t mq = mq_open(hbQueue(p.first).c_str(),
                               O_WRONLY | O_NONBLOCK);
            if (mq != -1) {
                mq_send(mq, (char*)&hb, sizeof(hb), 0);
                mq_close(mq);
            }
        }

        tick++;
        sleep(1);   // SINGLE clock in entire system
    }
}
