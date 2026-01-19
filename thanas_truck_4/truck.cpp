#include <iostream>
#include <map>
#include <iomanip>
#include <string>
#include <mqueue.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "common.h"

// ---------- follower ----------
void runFollower(int id) {

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

    // notify leader of join
    LeaderMsg join{LeaderMsgType::Join, id, 0.0};
    sendto(udpTx, &join, sizeof(join), 0,
           (sockaddr*)&leader, sizeof(leader));

    double speed = 20.0;
    double desiredDistance = 20.0;
    double actualDistance = 20.0;

    while (true) {
        
        // ---- read setpoint ----
        SetpointMsg sp;
        recvfrom(udpRx, &sp, sizeof(sp), MSG_DONTWAIT, nullptr, nullptr);
        desiredDistance = sp.desiredDistance;

        // ---- distance control (simple, stable) ----
        double error = actualDistance - desiredDistance;
        double accel = 0.1 * error;

        if (accel >  2.0) accel =  2.0;
        if (accel < -3.0) accel = -3.0;

        speed += accel;
        if (speed < 0.0) speed = 0.0;

        // ---- report distance ----
        LeaderMsg d{LeaderMsgType::Distance, id, actualDistance};
        sendto(udpTx, &d, sizeof(d), 0,
               (sockaddr*)&leader, sizeof(leader));

        std::cout << "[Follower " << id << "] "
                  << "speed=" << speed
                  << " dist=" << actualDistance << "\n";
    }
}

// ---------- leader ----------
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
    std::map<int, double> distances;

    while (true) {
        Heartbeat hb;
        mq_receive(mqHb, (char*)&hb, sizeof(hb), nullptr);

        if (std::cin.rdbuf()->in_avail()) {
            char c;
            std::cin >> c;
            if (c == '+') desiredDistance += 1.0;
            if (c == '-') desiredDistance -= 1.0;
        }

        SetpointMsg sp{desiredDistance};
        sendto(udpTx, &sp, sizeof(sp), 0,
               (sockaddr*)&tx, sizeof(tx));

        LeaderMsg msg;
        while (recvfrom(udpRx, &msg, sizeof(msg),
                        MSG_DONTWAIT, nullptr, nullptr) > 0) {
            if (msg.type == LeaderMsgType::Join)
                distances[msg.truckId] = 0.0;
            if (msg.type == LeaderMsgType::Distance)
                distances[msg.truckId] = msg.distance;
        }

        // ---- ASCII visualization ----
        std::cout << "\033[2J\033[H";
        std::cout << "+--------------+";
        for (auto& p : distances)
            std::cout << "                +--------------+";
        std::cout << "\n|  Truck 1 (L)  |";

        for (auto& p : distances) {
            std::cout << "   == "
                      << std::setw(4) << (int)p.second
                      << " m ==   | Truck "
                      << p.first << " |";
        }
        std::cout << "\n";
    }
}

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
}
