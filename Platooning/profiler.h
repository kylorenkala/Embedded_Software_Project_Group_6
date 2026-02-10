#ifndef PROFILER_H
#define PROFILER_H

#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <map>

class Profiler {
private:
    static std::map<std::string, double> bcet;
    static std::map<std::string, double> wcet;
    static std::map<std::string, double> totalTime;
    static std::map<std::string, int> sampleCount;

public:
    static void recordStart(const std::string& taskName, double& startOut) {
        startOut = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now().time_since_epoch()
        ).count();
    }

    static void recordEnd(const std::string& taskName, double startTime) {
        double now = std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now().time_since_epoch()
        ).count();
        double elapsed = now - startTime;

        if (bcet.find(taskName) == bcet.end()) {
            bcet[taskName] = elapsed;
            wcet[taskName] = elapsed;
        } else {
            bcet[taskName] = std::min(bcet[taskName], elapsed);
            wcet[taskName] = std::max(wcet[taskName], elapsed);
        }
        totalTime[taskName] += elapsed;
        sampleCount[taskName]++;
    }

    static void dumpResults(int truckId) {
        std::ofstream out("profile_truck_" + std::to_string(truckId) + ".csv");
        out << "Task,BCET_ms,WCET_ms,Avg_ms,Samples\n";
        for (auto& [name, bc] : bcet) {
            double avg = totalTime[name] / sampleCount[name];
            out << name << "," << bc << "," << wcet[name] << "," << avg << "," << sampleCount[name] << "\n";
        }
        out.close();
        std::cout << "[PROFILER] Results dumped to profile_truck_" << truckId << ".csv\n";
    }
};

// Static member definitions
inline std::map<std::string, double> Profiler::bcet;
inline std::map<std::string, double> Profiler::wcet;
inline std::map<std::string, double> Profiler::totalTime;
inline std::map<std::string, int>    Profiler::sampleCount;

#endif