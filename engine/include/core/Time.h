#pragma once
#include <chrono>

namespace Time
{
    inline double time()
    {
        auto currentTime = std::chrono::system_clock::now();
        auto duration = std::chrono::duration<double>(currentTime.time_since_epoch());

        return duration.count();
    }
}