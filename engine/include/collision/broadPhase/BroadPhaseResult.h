#pragma once

#include <collision/PotentialCollisionPair.h>

typedef std::vector<PotentialCollisionPair> PotentialCollisions;

struct BroadPhaseResult
{
    PotentialCollisions potentialCollisions;
    unsigned int numChecks = 0;
    double timeTaken;
};