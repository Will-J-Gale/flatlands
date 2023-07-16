#include <map>
#include <sstream>
#include <collision/broadPhase/SpatialHashDetection.h>
#include <collision/CollisionAlgorithms.h>

SpatialHashDetection::SpatialHashDetection(size_t cellSize, size_t hashTableSize)
{
    this->cellSize = cellSize;
    this->hashTableSize = hashTableSize;
}

size_t SpatialHashDetection::GenerateHash(const Vector2& p)
{
    Vector2 gridPos = CalculateGridPosition(p);
    size_t hash = (((int)gridPos.x * p1) ^ ((int)gridPos.y * p2)) % 2000;

    return hash;
}

Vector2 SpatialHashDetection::CalculateGridPosition(const Vector2& p)
{
    return Vector2(
        std::floor(p.x / cellSize) * cellSize,
        std::floor(p.y / cellSize) * cellSize
    );
}

std::vector<size_t> SpatialHashDetection::GenerateHashesForAABB(const AABB& aabb)
{
    std::vector<size_t> hashes;
    
    Vector2 topLeftGrid = CalculateGridPosition(aabb.min);
    Vector2 bottomRightGrid = CalculateGridPosition(aabb.max);

    for(size_t cellX = topLeftGrid.x; cellX <= bottomRightGrid.x; cellX += cellSize)
    {
        for(size_t cellY = topLeftGrid.y; cellY <= bottomRightGrid.y; cellY += cellSize)
        {
            hashes.push_back(GenerateHash(Vector2(cellX, cellY)));
        }
    }

    return hashes; 
}

BroadPhaseResult SpatialHashDetection::Execute(RigidBodies& bodies)
{
    BroadPhaseResult result;
    std::map<RigidBody*, AABB> bodyToAABB;
    std::map<size_t, std::vector<RigidBody*>> grid;
    // std::set<std::string> existingCollisionParis;    
    std::map<std::string, bool> existingCollisionParis;

    double start = Time::time();
    for (RigidBody* body : bodies)
    {
        AABB aabb = body->collider->GetAABB(&body->transform);
        bodyToAABB[body] = aabb;
        std::vector<size_t> hashes = GenerateHashesForAABB(aabb);

        for(size_t& hash : hashes)
        {
            if(grid.count(hash) == 0)
                grid[hash] = std::vector<RigidBody*>();
            
            grid[hash].push_back(body);
        }
    }

    for(auto it = grid.begin(); it != grid.end(); it++)
    {
        std::vector<RigidBody*> cellBodies = it->second;

        if(cellBodies.size() <= 1)
            continue;

        for(size_t i = 0; i < cellBodies.size(); i++)
        {
            for(size_t j = 0; j < cellBodies.size(); j++)
            {
                if(i == j || j < i)
                    continue;

                RigidBody* a = cellBodies[i];
                RigidBody* b = cellBodies[j];

                if((a->isStatic && b->isStatic) || (!a->isAwake && !b->isAwake))
                    continue;
                
                std::string collisionHash; 
                std::string collisionHashReverse; 
                collisionHash += a->hash;
                collisionHash += b->hash;
                collisionHashReverse += b->hash;
                collisionHashReverse += a->hash;

                //Checking if collision exists is SUPER slow
                if(existingCollisionParis.count(collisionHash) > 0 || existingCollisionParis.count(collisionHashReverse) > 0)
                    continue;
                else
                {
                    existingCollisionParis.insert({collisionHash, true});
                    existingCollisionParis.insert({collisionHashReverse, true});
                }

                AABB aAABB = bodyToAABB.count(a) > 0 ? bodyToAABB[a] : a->collider->GetAABB(&a->transform);
                AABB bAABB = bodyToAABB.count(b) > 0 ? bodyToAABB[b] : b->collider->GetAABB(&b->transform);
                    
                if(CollisionAlgorithms::TestAABBCollision(&aAABB, &bAABB))
                    result.potentialCollisions.emplace_back(a, b);
                
                result.numChecks += 1;
            }
        }
    }

    loginfo(existingCollisionParis.size());

    result.timeTaken = Time::time() - start;

    // std::string text = "Grid create time: " + std::to_string(result.timeTaken) + " Num potential collisions: " + std::to_string(result.potentialCollisions.size());
    // loginfo(text);

    return result;
}