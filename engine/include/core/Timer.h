#pragma once
#include <map>
#include <core/Time.h>

struct Timespan
{
    Timespan()
    {
        this->restart();
    }

    void restart()
    {
        this->start = Time::time();
    }

    void stop()
    {
        this->end = Time::time();
    }

    double getDuration()
    {
        return end - start;
    }

    double start;
    double end;
};

class Timer
{
public:
    static void start(std::string name)
    {
        if(timers.count(name) == 0)
            timers.insert({name, Timespan()});
        
        timers.at(name).restart();
    }
    static void stop(std::string name)
    {
        if(timers.count(name) > 0)
            timers.at(name).stop();
    }

    static inline std::map<std::string, Timespan> timers;
};