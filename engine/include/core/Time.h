#pragma once
#include <chrono>

namespace Time
{
    double time()
    {
        auto currentTime = std::chrono::system_clock::now();
        auto duration = std::chrono::duration<double>(currentTime.time_since_epoch());

        return duration.count();
    }
}