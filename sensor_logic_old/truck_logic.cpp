void TruckNode::runLogic() {
    auto lastTime = std::chrono::high_resolution_clock::now();

    while (true) {
        double t_logic_start;
        Profiler::recordStart("Logic", t_logic_start);

        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = now - lastTime;
        double dt = elapsed.count();
        lastTime = now;

        pthread_mutex_lock(&stateMutex);

        if (id < MAX_NODES) myMatrix[id][id]++;
        cleanupOldNeighbors();

        double currentTargetSpeed = 0.0;
        double gapFront = -1.0;

        if (isJamming) {
            jammingTimer += dt;
            if (jammingTimer < 10.0) {
                double blindSpeed = 50.0 / 3.6;
                physics.update(blindSpeed, dt);
                currentTargetSpeed = blindSpeed;
                if (id != 0) std::cout << " [JAMMED] Blind Cruise (" << std::fixed << std::setprecision(1) << jammingTimer << "s)\r";
            } else {
                physics.emergencyStop(dt);
                currentTargetSpeed = 0.0;
                if (id != 0) std::cout << " [JAMMED] TIMEOUT! Emergency Stop.\r";
                if (jammingTimer - dt < 10.0) {
                    logEvent("CRITICAL", "Jamming Timeout Exceeded (10s). Initiating Emergency Stop.");
                }
            }
        } else {
            jammingTimer = 0.0;

            // --- PREPARE DATA FOR OPENMP ---
            std::vector<PlatoonMessage> sensorData;
            std::vector<int> sensorIds;
            std::vector<double> preciseAges; // New Vector for smooth ages

            // Capture the current precise time for age calculation
            auto steadyNow = std::chrono::steady_clock::now();

            for (const auto& kv : neighbors) {
                sensorData.push_back(kv.second);
                sensorIds.push_back(kv.first);

                // --- CALCULATE PRECISE AGE ---
                // Calculate age in floating-point seconds (e.g., 0.123s)
                if (receptionTimes.count(kv.first)) {
                    double age = std::chrono::duration<double>(steadyNow - receptionTimes[kv.first]).count();
                    preciseAges.push_back(age);
                } else {
                    preciseAges.push_back(0.0);
                }
            }

            int dataSize = sensorData.size();

            #pragma omp parallel for
            for (int i = 0; i < dataSize; i++) {
                // Use the precise age we calculated earlier
                double age = preciseAges[i];

                // --- FIX: SMOOTH POSITION EXTRAPOLATION ---
                // Now using high-precision age, so the ghost moves smoothly
                if (age > 0.0 && age < FAILURE_TIMEOUT) {
                    sensorData[i].position += (sensorData[i].speed * age);
                }

                if (age > SIGNAL_TIMEOUT) {
                    double relativePos = sensorData[i].position - physics.getPosition();
                    if (relativePos > 0) {
                        sensorData[i].speed = 0.0;
                        sensorData[i].emergencyBrake = true;
                    }
                }
            }

            std::map<int, PlatoonMessage> processedNeighbors;
            for (int i = 0; i < dataSize; i++) {
                processedNeighbors[sensorIds[i]] = sensorData[i];
                if (sensorIds[i] != id) {
                    double dist = sensorData[i].position - physics.getPosition();
                    if (dist > 0 && (gapFront < 0 || dist < gapFront)) {
                        gapFront = dist;
                    }
                }
            }

            double targetSpeed = controller.calculateTargetSpeed(
                id, physics.getPosition(), physics.getSpeed(),
                processedNeighbors, isDecoupled, emergencyBrake, targetPlatoonSize
            );

            currentTargetSpeed = targetSpeed;
            physics.update(targetSpeed, dt);
        }

        logData(currentTargetSpeed, gapFront);
        logMatrix();

        pthread_mutex_unlock(&stateMutex);

        Profiler::recordEnd("Logic", t_logic_start);
        usleep(50000);
    }
}
