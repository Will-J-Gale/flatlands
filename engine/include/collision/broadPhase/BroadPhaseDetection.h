#pragma once

#include <vector>
#include <functional>
#include <collision/PotentialCollisionPair.h>
#include <collision/broadPhase/BroadPhaseResult.h>
#include <core/Time.h>
#include <RigidBody.h>

typedef std::vector<RigidBody*> RigidBodies;

class BroadPhaseDetection
{
public:
    virtual BroadPhaseResult execute(const RigidBodies& bodies) = 0;
};