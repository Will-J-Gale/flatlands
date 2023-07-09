#pragma once
struct Metrics
{
    //Counts
    unsigned int numEntities = 0;
    unsigned int broadPhaseChecks = 0;
    unsigned int narrowPhaseChecks = 0;

    //Timings
    double worldStepTime = 0.0;
    double moveBodiesTime = 0.0;
    double broadPhaseTime = 0.0;
    double narrowPhaseTime = 0.0;
    double resolveCollisionsTime = 0.0;

    void reset()
    {
        numEntities = 0;
        broadPhaseChecks = 0;
        narrowPhaseChecks = 0;
        worldStepTime = 0.0;
        moveBodiesTime = 0.0;
        broadPhaseTime = 0.0;
        narrowPhaseTime = 0.0;
        resolveCollisionsTime = 0.0;
    };
};