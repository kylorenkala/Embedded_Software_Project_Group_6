#include <iostream>
#include <map>
#include <iomanip>
#include <string>
#include <mqueue.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "common.h"

// ========== Helpers ==========

std::string hbQueue(int id) {
    return "/mq_hb_" + std::to_string(id);
}

std::string sensorQueue(int id) {
    return "/mq_sensor_" + std::to_string(id);
}

// ========== Follower ==========

void runFollower(int id) {
    mqd_t mqHb = mq_open(hbQueue(id).c_str(), O_RDONLY | O_NONBLOCK);
    mqd_t mqSn = mq_open(sensorQueue(id).c_str(), O_RDONLY | O_NONBLOCK);

    int udpTx = socket(AF_INET, SOCK_DGRAM, 0);
    int udpRx = socket(AF_INET, SOCK_DGRAM, 0);

    sockaddr_in rx{};
    rx.sin_family = AF_INET;
    rx.sin_port = htons(6000);
    rx.sin_addr.s_addr = INADDR_ANY;
    bind(udpRx, (sockaddr*)&rx, sizeof(rx));

    sockaddr_in leader{};
    leader.sin_family = AF_INET;
    leader.sin_port = htons(6001);
    inet_pton(AF_INET, "127.0.0.1", &leader.sin_addr);

    // Join platoon
    LeaderMsg join{LeaderMsgType::Join, id, 0.0};
    sendto(udpTx, &join, sizeof(join), 0,
           (sockaddr*)&leader, sizeof(leader));
    std::cout << "[Follower " << id << "] Joining platoon\n";

    double speed = 0.0;
    double desiredDistance = 20.0;
    double actualDistance = 20.0;
    bool emergencyMode = false;
    int missedHeartbeats = 0;

    while (true) {
        // Check heartbeat
        Heartbeat hb;
        if (mq_receive(mqHb, (char*)&hb, sizeof(hb), nullptr) > 0) {
            missedHeartbeats = 0;
        } else {
            missedHeartbeats++;
            if (missedHeartbeats >= HEARTBEAT_TIMEOUT) {
                std::cerr << "[Follower " << id << "] Lost communication! Emergency stop\n";
                emergencyMode = true;
            }
        }

        // Read sensors
        SensorMsg s;
        while (mq_receive(mqSn, (char*)&s, sizeof(s), nullptr) > 0) {
            actualDistance = s.distanceToFront;
            
            // Obstacle detected?
            if (s.obstacleDetected && !emergencyMode) {
                std::cout << "[Follower " << id << "] OBSTACLE! Emergency brake\n";
                emergencyMode = true;
                LeaderMsg brake{LeaderMsgType::EmergencyBrake, id, actualDistance};
                sendto(udpTx, &brake, sizeof(brake), 0,
                       (sockaddr*)&leader, sizeof(leader));
            }
        }

        // Read leader commands
        SetpointMsg sp;
        if (recvfrom(udpRx, &sp, sizeof(sp), MSG_DONTWAIT, nullptr, nullptr) > 0) {
            desiredDistance = sp.desiredDistance;
            
            if (sp.emergencyBrake && !emergencyMode) {
                std::cout << "[Follower " << id << "] Leader emergency signal\n";
                emergencyMode = true;
            }
        }

        // Safety check: collision risk?
        if (!emergencyMode && isCollisionRisk(actualDistance, speed)) {
            std::cout << "[Follower " << id << "] Collision risk! Braking\n";
            emergencyMode = true;
        }

        // Control logic
        if (emergencyMode) {
            // Emergency brake
            speed -= EMERGENCY_DECEL * 0.5;
            if (speed <= 0.1) {
                speed = 0.0;
                emergencyMode = false; // Can resume
            }
        } else {
            // Normal distance control
            double error = actualDistance - desiredDistance;
            double accel = 0.15 * error;

            if (accel >  2.0) accel =  2.0;
            if (accel < -3.0) accel = -3.0;

            speed += accel;
            if (speed < 0.0) speed = 0.0;
            if (speed > 25.0) speed = 25.0;
        }

        // Report to leader
        LeaderMsg dist{LeaderMsgType::Distance, id, actualDistance};
        sendto(udpTx, &dist, sizeof(dist), 0,
               (sockaddr*)&leader, sizeof(leader));

        // Display status
        std::string status = emergencyMode ? "EMERGENCY" : "NORMAL";
        std::string safe = isSafeDistance(actualDistance) ? "SAFE" : "UNSAFE";
        
        std::cout << "[Follower " << id << "] "
                  << "speed=" << std::fixed << std::setprecision(1) << speed
                  << " dist=" << actualDistance 
                  << " [" << status << "/" << safe << "]\n";

        sleep(1);
    }
}

// ========== Leader ==========

void runLeader(int id) {
    mqd_t mqHb = mq_open(hbQueue(id).c_str(), O_RDONLY);

    int udpTx = socket(AF_INET, SOCK_DGRAM, 0);
    int udpRx = socket(AF_INET, SOCK_DGRAM, 0);

    sockaddr_in tx{};
    tx.sin_family = AF_INET;
    tx.sin_port = htons(6000);
    inet_pton(AF_INET, "127.0.0.1", &tx.sin_addr);

    sockaddr_in rx{};
    rx.sin_family = AF_INET;
    rx.sin_port = htons(6001);
    rx.sin_addr.s_addr = INADDR_ANY;
    bind(udpRx, (sockaddr*)&rx, sizeof(rx));

    double desiredDistance = 20.0;
    bool emergencyBrake = false;
    std::map<int, double> distances;

    std::cout << "[Leader " << id << "] Commands: +/- distance, e=emergency, r=reset, q=quit\n";

    while (true) {
        Heartbeat hb;
        mq_receive(mqHb, (char*)&hb, sizeof(hb), nullptr);

        // User input
        if (std::cin.rdbuf()->in_avail()) {
            char c;
            std::cin >> c;
            if (c == '+') desiredDistance += 2.0;
            if (c == '-') desiredDistance -= 2.0;
            if (c == 'e') {
                emergencyBrake = true;
                std::cout << "[Leader] EMERGENCY BRAKE ACTIVATED\n";
            }
            if (c == 'r') {
                emergencyBrake = false;
                std::cout << "[Leader] Emergency reset\n";
            }
            if (c == 'q') {
                std::cout << "[Leader] Shutting down\n";
                break;
            }
        }

        // Send commands to followers
        SetpointMsg sp{desiredDistance, emergencyBrake};
        sendto(udpTx, &sp, sizeof(sp), 0,
               (sockaddr*)&tx, sizeof(tx));

        // Receive follower messages
        LeaderMsg msg;
        while (recvfrom(udpRx, &msg, sizeof(msg),
                        MSG_DONTWAIT, nullptr, nullptr) > 0) {
            
            if (msg.type == LeaderMsgType::Join) {
                distances[msg.truckId] = 0.0;
                std::cout << "[Leader] Truck " << msg.truckId << " joined\n";
            }
            else if (msg.type == LeaderMsgType::Leave) {
                distances.erase(msg.truckId);
                std::cout << "[Leader] Truck " << msg.truckId << " left\n";
            }
            else if (msg.type == LeaderMsgType::Distance) {
                distances[msg.truckId] = msg.distance;
            }
            else if (msg.type == LeaderMsgType::EmergencyBrake) {
                if (!emergencyBrake) {
                    std::cout << "[Leader] Truck " << msg.truckId 
                              << " triggered emergency brake!\n";
                    emergencyBrake = true;
                }
            }
        }

        // Display platoon
        std::cout << "\033[2J\033[H"; // Clear screen
        std::cout << "=== PLATOON STATUS ===\n";
        std::cout << "Desired Distance: " << desiredDistance << "m | ";
        std::cout << "Emergency: " << (emergencyBrake ? "ACTIVE" : "OFF") << "\n\n";

        std::cout << "+--------------+";
        for (auto& p : distances)
            std::cout << "           +--------------+";
        std::cout << "\n|  Leader " << id << "   |";

        for (auto& p : distances) {
            std::cout << "  " << std::setw(4) << (int)p.second << "m  "
                      << "| Truck " << p.first << "     |";
        }
        std::cout << "\n\n";
    }

    // Cleanup
    mq_close(mqHb);
    close(udpTx);
    close(udpRx);
}

// ========== Main ==========

int main() {
    int id;
    char role;

    std::cout << "Truck ID: ";
    std::cin >> id;
    std::cout << "Role (l/f): ";
    std::cin >> role;

    if (role == 'l')
        runLeader(id);
    else
        runFollower(id);

    // Cleanup queues
    mq_unlink(hbQueue(id).c_str());
    mq_unlink(sensorQueue(id).c_str());

    return 0;
}