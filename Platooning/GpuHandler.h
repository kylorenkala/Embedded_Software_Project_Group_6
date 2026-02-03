#ifndef GPU_HANDLER_H
#define GPU_HANDLER_H

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <algorithm> // For std::transform

#define CL_TARGET_OPENCL_VERSION 300

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif

class GpuHandler {
private:
    cl_context context;
    cl_command_queue queue;
    cl_program program;
    cl_kernel kernel;

    // GPU Buffers (Now sized for float)
    cl_mem d_pos, d_spd, d_age, d_outPos, d_outSpeed, d_outBrake;
    size_t maxDataSize;

public:
    GpuHandler(int maxNeighbors = 100) : maxDataSize(maxNeighbors) {
        cl_int err;
        cl_uint numPlatforms;
        cl_platform_id* platforms = NULL;

        clGetPlatformIDs(0, NULL, &numPlatforms);
        platforms = (cl_platform_id*)malloc(sizeof(cl_platform_id) * numPlatforms);
        clGetPlatformIDs(numPlatforms, platforms, NULL);

        cl_device_id device = NULL;
        bool deviceFound = false;

        for (int i = 0; i < numPlatforms; i++) {
            err = clGetDeviceIDs(platforms[i], CL_DEVICE_TYPE_GPU, 1, &device, NULL);
            if (err == CL_SUCCESS) {
                deviceFound = true;
                char name[128];
                clGetDeviceInfo(device, CL_DEVICE_NAME, 128, name, NULL);
                std::cout << "[GPU] Initialized on: " << name << std::endl;
                break;
            }
        }
        free(platforms);

        if (!deviceFound) {
            std::cerr << "CRITICAL ERROR: No OpenCL GPU found!" << std::endl;
            exit(1);
        }

        context = clCreateContext(NULL, 1, &device, NULL, NULL, &err);

        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        queue = clCreateCommandQueue(context, device, 0, &err);
        #pragma GCC diagnostic pop

        std::ifstream file("fusion.cl");
        if (!file.is_open()) {
            std::cerr << "ERROR: Could not open fusion.cl" << std::endl;
            exit(1);
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string src = buffer.str();
        const char* srcPtr = src.c_str();

        program = clCreateProgramWithSource(context, 1, &srcPtr, NULL, &err);
        err = clBuildProgram(program, 1, &device, NULL, NULL, NULL);

        if (err != CL_SUCCESS) {
            char log[16384];
            clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG, sizeof(log), log, NULL);
            std::cerr << "Kernel Build Error:\n" << log << std::endl;
        }

        kernel = clCreateKernel(program, "sensorFusion", &err);

        // --- ALLOCATE BUFFERS AS FLOAT (sizeof(float)) ---
        size_t fSize = maxDataSize * sizeof(float);
        size_t iSize = maxDataSize * sizeof(int);

        d_pos = clCreateBuffer(context, CL_MEM_READ_ONLY, fSize, NULL, NULL);
        d_spd = clCreateBuffer(context, CL_MEM_READ_ONLY, fSize, NULL, NULL);
        d_age = clCreateBuffer(context, CL_MEM_READ_ONLY, fSize, NULL, NULL);
        d_outPos = clCreateBuffer(context, CL_MEM_WRITE_ONLY, fSize, NULL, NULL);
        d_outSpeed = clCreateBuffer(context, CL_MEM_WRITE_ONLY, fSize, NULL, NULL);
        d_outBrake = clCreateBuffer(context, CL_MEM_WRITE_ONLY, iSize, NULL, NULL);
    }

    ~GpuHandler() {
        clReleaseMemObject(d_pos); clReleaseMemObject(d_spd); clReleaseMemObject(d_age);
        clReleaseMemObject(d_outPos); clReleaseMemObject(d_outSpeed); clReleaseMemObject(d_outBrake);
        clReleaseKernel(kernel); clReleaseProgram(program);
        clReleaseCommandQueue(queue); clReleaseContext(context);
    }

    void runSensorFusion(const std::vector<double>& h_pos, const std::vector<double>& h_spd, const std::vector<double>& h_age,
                         std::vector<double>& out_pos, std::vector<double>& out_spd, std::vector<int>& out_brake,
                         double myPos, double sigTimeout, double failTimeout) {
        int n = h_pos.size();
        if(n == 0) return;

        // --- CONVERT DOUBLE TO FLOAT ---
        std::vector<float> f_pos(h_pos.begin(), h_pos.end());
        std::vector<float> f_spd(h_spd.begin(), h_spd.end());
        std::vector<float> f_age(h_age.begin(), h_age.end());

        float f_myPos = (float)myPos;
        float f_sigTimeout = (float)sigTimeout;
        float f_failTimeout = (float)failTimeout;

        size_t fSize = n * sizeof(float);
        size_t iSize = n * sizeof(int);

        // Write Float Buffers
        clEnqueueWriteBuffer(queue, d_pos, CL_TRUE, 0, fSize, f_pos.data(), 0, NULL, NULL);
        clEnqueueWriteBuffer(queue, d_spd, CL_TRUE, 0, fSize, f_spd.data(), 0, NULL, NULL);
        clEnqueueWriteBuffer(queue, d_age, CL_TRUE, 0, fSize, f_age.data(), 0, NULL, NULL);

        clSetKernelArg(kernel, 0, sizeof(cl_mem), &d_pos);
        clSetKernelArg(kernel, 1, sizeof(cl_mem), &d_spd);
        clSetKernelArg(kernel, 2, sizeof(cl_mem), &d_age);
        clSetKernelArg(kernel, 3, sizeof(cl_mem), &d_outPos);
        clSetKernelArg(kernel, 4, sizeof(cl_mem), &d_outSpeed);
        clSetKernelArg(kernel, 5, sizeof(cl_mem), &d_outBrake);
        clSetKernelArg(kernel, 6, sizeof(int), &n);
        clSetKernelArg(kernel, 7, sizeof(float), &f_myPos);         // Pass float
        clSetKernelArg(kernel, 8, sizeof(float), &f_sigTimeout);    // Pass float
        clSetKernelArg(kernel, 9, sizeof(float), &f_failTimeout);   // Pass float

        size_t globalSize = n;
        clEnqueueNDRangeKernel(queue, kernel, 1, NULL, &globalSize, NULL, 0, NULL, NULL);

        // Read Back Floats
        std::vector<float> f_outPos(n);
        std::vector<float> f_outSpeed(n);
        out_brake.resize(n);

        clEnqueueReadBuffer(queue, d_outPos, CL_TRUE, 0, fSize, f_outPos.data(), 0, NULL, NULL);
        clEnqueueReadBuffer(queue, d_outSpeed, CL_TRUE, 0, fSize, f_outSpeed.data(), 0, NULL, NULL);
        clEnqueueReadBuffer(queue, d_outBrake, CL_TRUE, 0, iSize, out_brake.data(), 0, NULL, NULL);

        // --- CONVERT FLOAT BACK TO DOUBLE ---
        out_pos.assign(f_outPos.begin(), f_outPos.end());
        out_spd.assign(f_outSpeed.begin(), f_outSpeed.end());
    }
};

#endif