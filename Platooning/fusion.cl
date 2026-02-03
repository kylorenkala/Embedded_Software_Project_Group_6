// File: fusion.cl
// Uses FLOAT (32-bit) for compatibility with Intel Iris Xe

__kernel void sensorFusion(
    __global const float* positions,   // Changed to float
    __global const float* speeds,      // Changed to float
    __global const float* ages,        // Changed to float
    __global float* outPos,            // Changed to float
    __global float* outSpeed,          // Changed to float
    __global int* outBrake,
    int numNeighbors,
    float myPos,                       // Changed to float
    float signalTimeout,               // Changed to float
    float failureTimeout               // Changed to float
) {
    int i = get_global_id(0);

    if (i < numNeighbors) {
        float age = ages[i];
        float currentPos = positions[i];
        float currentSpeed = speeds[i];

        // 1. Position Extrapolation
        if (age > 0.0f && age < failureTimeout) {
            currentPos += (currentSpeed * age);
        }

        // Write results
        outPos[i] = currentPos;
        outSpeed[i] = currentSpeed;
        outBrake[i] = 0;

        // 2. Ghost Detection
        if (age > signalTimeout) {
            float relativePos = currentPos - myPos;

            if (relativePos > 0.0f) {
                outSpeed[i] = 0.0f;
                outBrake[i] = 1;
            }
        }
    }
}