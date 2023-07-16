#include <sstream>
#include <set>
#include <collision/broadPhase/QuadTreeDetection.h>
#include <core/Time.h>
#include <core/Logger.h>

QuadTreeDetection::QuadTreeDetection(AABB rootBounds, size_t nodeCapacity)
{
    this->rootBounds = rootBounds;
    this->nodeCapacity = nodeCapacity;
}
BroadPhaseResult QuadTreeDetection::Execute(RigidBodies& bodies)
{
    double start = Time::time();
    BroadPhaseResult result;
    quadTree = std::make_unique<QuadTree>(rootBounds, nodeCapacity);

    double quadTreeStart = Time::time();
    for(RigidBody* body : bodies)
    {
        quadTree->Insert(body);
    }
    double quadTreeEnd = Time::time() - quadTreeStart;
    double collisionCheckStart = Time::time();

    std::set<std::string> existingCollisionParis;

    for(RigidBody* body : bodies)
    {
        std::vector<RigidBody*> potentialCollisions;
        AABB aabb = body->collider->GetAABB(&body->transform);
        quadTree->Query(aabb, potentialCollisions);

        std::ostringstream bodyMemoryAddressStream;
        bodyMemoryAddressStream << body;
        std::string bodyMemoryAddress = bodyMemoryAddressStream.str();

        for(RigidBody* other : potentialCollisions)
        {
            if(other != body)
            {
                std::ostringstream otherMemoryAddressStream;
                otherMemoryAddressStream << other;
                std::string otherMemoryAddress = otherMemoryAddressStream.str();

                std::string collisionPairHash = bodyMemoryAddress+otherMemoryAddress;
                std::string collisionPairReverseHash = otherMemoryAddress+bodyMemoryAddress;

                if(existingCollisionParis.count(collisionPairHash) > 0 || existingCollisionParis.count(collisionPairReverseHash) > 0)
                    continue;

                result.numChecks++;
                result.potentialCollisions.emplace_back(body, other);
                existingCollisionParis.insert(collisionPairHash);
                existingCollisionParis.insert(collisionPairReverseHash);
            }
        }
    }
    double collisionCheckEnd = Time::time() - collisionCheckStart;
    result.timeTaken = Time::time() - start;
    return result;
}

QuadTree* QuadTreeDetection::GetQuadTree()
{
    return quadTree.get();
}