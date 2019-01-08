#ifndef EBV_BENCHMARKING_H
#define EBV_BENCHMARKING_H

#include <iostream>
#include <chrono>
#include <thread>

#include <EBV_LaserController.h>

struct Timer
{
    std::chrono::high_resolution_clock::time_point start,end;
    std::chrono::duration<float> duration;

    Timer()
    {
        start = std::chrono::high_resolution_clock::now();
    }

    ~Timer()
    {
        end = std::chrono::high_resolution_clock::now();
        duration = end - start;
        float ms = duration.count() * 1000.0f;
        std::cout << "Timer took " << ms << "\n\r";
    }
};

class LaserTest
{
public:
    LaserTest(LaserController* laser);
    ~LaserTest();

    void computeAngularVelocity();
    LaserController* m_laser;

};

#endif // EBV_BENCHMARKING_H
