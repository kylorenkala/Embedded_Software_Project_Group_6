#include <iostream>
#include <unordered_map>
#include <vector>
#include <string>
#include <mqueue.h>
#include <unistd.h>
#include <fstream>
#include "common.h"

// ========== Helpers ==========

std::string hbQueue(int id) {
    return "/mq_hb_" + std::to_string(id);
}

std::string sensorQueue(int id) {
    return "/mq_sensor_" + std::to_string(id);
}

struct WorldTruck {
    double position;
    bool obstacleAhead;
};

// ========== Simple Logger ==========

class Logger {
    std::ofstream file;
public:
    Logger() {
        file.open("platoon_log.txt", std::ios::app);
    }
    
    void log(const std::string& msg) {
        if (file.is_open()) {
            file << msg << "\n";
            file.flush();
        }
    }
    
    ~Logger() {
        if (file.is_open()) file.close();
    }
};

// ========== Main Frame ==========

int main() {
    std::unordered_map<int, WorldTruck> trucks;
    std::unordered_map<int, mqd_t> hbQueues;
    std::unordered_map<int, mqd_t> sensorQueues;
    Logger logger;
    
    uint64_t tick = 0;
    int obstacleAt = -1;

    std::cout << "=== MainFrame Simulation ===\n";
    std::cout << "Commands:\n";
    std::cout << "  <ID>   - Register truck\n";
    std::cout << "  u <ID> - Unregister truck\n";
    std::cout << "  o <ID> - Obstacle at truck\n";
    std::cout << "  c      - Clear obstacle\n\n";

    while (true) {
        // Handle commands
        if (std::cin.rdbuf()->in_avail()) {
            char cmd;
            std::cin >> cmd;

            if (cmd == 'u') {
                int id;
                std::cin >> id;
                
                trucks.erase(id);
                if (hbQueues.count(id)) {
                    mq_close(hbQueues[id]);
                    mq_unlink(hbQueue(id).c_str());
                    hbQueues.erase(id);
                }
                if (sensorQueues.count(id)) {
                    mq_close(sensorQueues[id]);
                    mq_unlink(sensorQueue(id).c_str());
                    sensorQueues.erase(id);
                }
                
                std::cout << "Unregistered truck " << id << "\n";
                logger.log("Truck " + std::to_string(id) + " unregistered");
            }
            else if (cmd == 'o') {
                int id;
                std::cin >> id;
                if (trucks.count(id)) {
                    obstacleAt = id;
                    trucks[id].obstacleAhead = true;
                    std::cout << "Obstacle placed at truck " << id << "\n";
                    logger.log("OBSTACLE at truck " + std::to_string(id));
                }
            }
            else if (cmd == 'c') {
                obstacleAt = -1;
                for (auto& p : trucks) {
                    p.second.obstacleAhead = false;
                }
                std::cout << "Obstacle cleared\n";
                logger.log("Obstacle cleared");
            }
            else if (std::isdigit(cmd)) {
                std::cin.putback(cmd);
                int id;
                std::cin >> id;

                trucks[id] = {0.0, false};

                mq_attr attr{};
                attr.mq_maxmsg = 10;

                attr.mq_msgsize = sizeof(Heartbeat);
                hbQueues[id] = mq_open(hbQueue(id).c_str(),
                        O_CREAT | O_WRONLY | O_NONBLOCK,
                        0666, &attr);

                attr.mq_msgsize = sizeof(SensorMsg);
                sensorQueues[id] = mq_open(sensorQueue(id).c_str(),
                        O_CREAT | O_WRONLY | O_NONBLOCK,
                        0666, &attr);

                std::cout << "Registered truck " << id << "\n";
                logger.log("Truck " + std::to_string(id) + " registered");
            }
        }

        // Update positions
        std::vector<int> order;
        for (auto& p : trucks)
            order.push_back(p.first);
        std::sort(order.begin(), order.end());

        for (size_t i = 0; i < order.size(); ++i)
            trucks[order[i]].position = i * 25.0;

        // Send sensors
        for (size_t i = 1; i < order.size(); ++i) {
            int currId = order[i];
            int frontId = order[i - 1];
            
            double distance = trucks[frontId].position - trucks[currId].position;

            SensorMsg s{};
            s.distanceToFront = distance;
            s.obstacleDetected = trucks[currId].obstacleAhead;

            mqd_t mq = sensorQueues[currId];
            if (mq != -1) {
                mq_send(mq, (char*)&s, sizeof(s), 0);
            }

            // Safety warning
            if (distance < MIN_SAFE_DISTANCE) {
                std::cerr << "WARNING: Truck " << currId 
                          << " too close (" << distance << "m)\n";
                logger.log("SAFETY: Truck " + std::to_string(currId) + 
                          " below minimum distance");
            }
        }

        // Send heartbeats
        for (auto& p : trucks) {
            Heartbeat hb{tick};
            mqd_t mq = hbQueues[p.first];
            if (mq != -1) {
                mq_send(mq, (char*)&hb, sizeof(hb), 0);
            }
        }

        // Display
        std::cout << "[Tick " << tick << "] Trucks: " << trucks.size();
        if (obstacleAt != -1) {
            std::cout << " | OBSTACLE at " << obstacleAt;
        }
        std::cout << "\n";

        tick++;
        sleep(1);
    }
}