#pragma once

#include <memory>
#include <set>
#include <collision/broadPhase/BroadPhaseDetection.h>
#include <collision/broadPhase/BroadPhaseResult.h>
#include <RigidBody.h>

class SpatialHashDetection : public BroadPhaseDetection
{
public:
    SpatialHashDetection(size_t cellSize, size_t hashTabeSize = 5000);
    BroadPhaseResult Execute(RigidBodies& bodies) override;

private:
    Vector2 CalculateGridPosition(const Vector2& p);
    size_t GenerateHash(const Vector2& p);
    std::vector<size_t> GenerateHashesForAABB(const AABB& aabb);

private:
    size_t cellSize;
    size_t hashTableSize;
    //Constants from http://www.beosil.com/download/CollisionDetectionHashing_VMV03.pdf
    static const size_t p1 = 73856093;
    static const size_t p2 = 19349663;
};