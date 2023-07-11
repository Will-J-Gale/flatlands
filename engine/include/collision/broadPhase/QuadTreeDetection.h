#pragma once

#include <collision/broadPhase/BroadPhaseDetection.h>
#include <collision/broadPhase/QuadTree.h>

class QuadTreeDetection : public BroadPhaseDetection
{
public:
    BroadPhaseResult execute(const RigidBodies& bodies) override;

private:
    bool isSubdivided = false;
    QuadTree quadTree;
};