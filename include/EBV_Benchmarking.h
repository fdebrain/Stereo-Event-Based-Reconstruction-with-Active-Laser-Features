#include <iostream>
#include <chrono>
#include <thread>

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
