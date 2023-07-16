#pragma once

#include <memory>
#include <vector>
#include <math/Vector2.h>
#include <math/AABB.h>
#include <RigidBody.h>
#include <collision/CollisionAlgorithms.h>
#include <RigidBody.h>
#include <Transform.h>

class QuadTree
{
public:
    QuadTree(){}
    QuadTree(AABB boundary, size_t nodeCapacity=4)
    {
        this->boundary = boundary;
        this->nodeCapacity = nodeCapacity;
    };

    ~QuadTree()
    {
        int i = 0;
    }

    void Insert(RigidBody* body)
    {
        AABB aabb = body->collider->GetAABB(&body->transform);
        if(!CollisionAlgorithms::TestAABBCollision(&aabb, &boundary))
            return;

        if(bodies.size() < this->nodeCapacity)
            bodies.push_back(body);
        else
        {
            if(!divided)
                Subdivide();
            
            topLeft->Insert(body);
            topRight->Insert(body);
            bottomLeft->Insert(body);
            bottomRight->Insert(body);
        }
    }

    void Query(const AABB& queryBounds, std::vector<RigidBody*>& result)
    {
        if(CollisionAlgorithms::TestAABBCollision(&queryBounds, &boundary))
        {
            for(RigidBody* body : bodies)
            {
                AABB aabb = body->collider->GetAABB(&body->transform);
                if(CollisionAlgorithms::TestAABBCollision(&aabb, &queryBounds))
                    result.push_back(body);
            }

            if(divided)
            {
                topLeft->Query(queryBounds, result);
                topRight->Query(queryBounds, result);
                bottomLeft->Query(queryBounds, result);
                bottomRight->Query(queryBounds, result);
            }
        }

    }

    void Query(RigidBody* body, std::vector<RigidBody*>& result)
    {
        AABB aabb = body->collider->GetAABB(&body->transform);
        if(CollisionAlgorithms::TestAABBCollision(&aabb, &boundary))
        {
            for(RigidBody* other : bodies)
            {
                // if(!body->isAwake && !other->isAwake)
                //     continue;

                AABB otherAABB = other->collider->GetAABB(&body->transform);
                if(CollisionAlgorithms::TestAABBCollision(&aabb, &otherAABB))
                    result.push_back(body);
            }

            if(divided)
            {
                topLeft->Query(body, result);
                topRight->Query(body, result);
                bottomLeft->Query(body, result);
                bottomRight->Query(body, result);
            }
        }

    }

    bool IsLeaf()
    {
        return !divided;
    }

    void GetTreeBounds(std::vector<AABB>* boundsList)
    {
        boundsList->push_back(boundary);

        if(divided)
        {
            topLeft->GetTreeBounds(boundsList);
            topRight->GetTreeBounds(boundsList);
            bottomLeft->GetTreeBounds(boundsList);
            bottomRight->GetTreeBounds(boundsList);
        }
    }

private:
    void Subdivide()
    {
        divided = true;

        Vector2 min = boundary.min;
        Vector2 max = boundary.max;
        float halfWidth = boundary.width / 2.0f;
        float halfHeight = boundary.height / 2.0f;

        AABB tl = AABB(min, Vector2(min.x + halfWidth, min.y + halfHeight));
        AABB tr = AABB(Vector2(min.x + halfWidth, min.y), Vector2(max.x, max.y - halfHeight));
        AABB bl = AABB(Vector2(min.x, min.y + halfHeight), Vector2(min.x + halfWidth, max.y));
        AABB br = AABB(Vector2(min.x + halfWidth, min.y + halfHeight), max);

        topLeft = std::make_unique<QuadTree>(tl, this->nodeCapacity);
        topRight = std::make_unique<QuadTree>(tr, this->nodeCapacity);
        bottomLeft = std::make_unique<QuadTree>(bl, this->nodeCapacity);
        bottomRight = std::make_unique<QuadTree>(br, this->nodeCapacity);
    }

private:
    size_t nodeCapacity = 1;
    bool divided = false;
    AABB boundary;
    std::vector<RigidBody*> bodies;
    std::unique_ptr<QuadTree> topLeft = nullptr;
    std::unique_ptr<QuadTree> topRight = nullptr;
    std::unique_ptr<QuadTree> bottomLeft = nullptr;
    std::unique_ptr<QuadTree> bottomRight = nullptr;
};