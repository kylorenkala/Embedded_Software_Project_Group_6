// File: fusion.cl
// This code runs on the GPU.

__kernel void sensorFusion(
    __global const double* positions,
    __global const double* speeds,
    __global const double* ages,
    __global double* outPos,
    __global double* outSpeed,
    __global int* outBrake,
    int numNeighbors,
    double myPos,
    double signalTimeout,
    double failureTimeout
) {
    // Get unique ID for this thread (0, 1, 2, ...)
    int i = get_global_id(0);

    // Boundary check
    if (i < numNeighbors) {
        double age = ages[i];
        double currentPos = positions[i];
        double currentSpeed = speeds[i];

        // 1. Position Extrapolation (Predict where neighbor is now)
        if (age > 0.0 && age < failureTimeout) {
            currentPos += (currentSpeed * age);
        }

        // Write results to output buffers
        outPos[i] = currentPos;
        outSpeed[i] = currentSpeed;
        outBrake[i] = 0; // Default: No brake

        // 2. Ghost Detection (Check for timeouts)
        if (age > signalTimeout) {
            double relativePos = currentPos - myPos;

            // Only panic if the ghost is AHEAD of us
            if (relativePos > 0) {
                outSpeed[i] = 0.0;    // Assume stopped
                outBrake[i] = 1;      // Flag emergency brake
            }
        }
    }
}