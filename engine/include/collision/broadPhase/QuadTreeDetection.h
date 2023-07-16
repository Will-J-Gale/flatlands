#pragma once

#include <memory>
#include <collision/broadPhase/BroadPhaseDetection.h>
#include <collision/broadPhase/QuadTree.h>

class QuadTreeDetection : public BroadPhaseDetection
{
public:
    QuadTreeDetection(AABB rootBounds, size_t nodeCapacity);
    BroadPhaseResult Execute(RigidBodies& bodies) override;
    QuadTree* GetQuadTree();

private:
    bool isSubdivided = false;
    AABB rootBounds;
    size_t nodeCapacity = 1;
    std::unique_ptr<QuadTree> quadTree = nullptr;
};