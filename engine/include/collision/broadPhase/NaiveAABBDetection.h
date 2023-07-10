#pragma once

#include <collision/broadPhase/BroadPhaseDetection.h>
#include <collision/broadPhase/BroadPhaseResult.h>

class NaiveAABBDetection : public BroadPhaseDetection
{
public:
    BroadPhaseResult execute(const RigidBodies& bodies) override;
};