#include <memory>
#include <vector>
#include <math/Vector2.h>
#include <math/AABB.h>
#include <RigidBody.h>
#include <collision/CollisionAlgorithms.h>

class QuadTree
{
public:
    QuadTree(AABB boundary, size_t capacity=4)
    {
        this->boundary = boundary;
        this->capacity = capacity;
    };

    ~QuadTree()
    {
        int i = 0;
    }

    void Insert(Vector2 point)
    {
        if(!CollisionAlgorithms::TestPointAABBCollision(point, boundary))
            return;

        if(points.size() < this->capacity)
            points.push_back(point);
        else
        {
            if(!divided)
                Subdivide();
            
            topLeft->Insert(point);
            topRight->Insert(point);
            bottomLeft->Insert(point);
            bottomRight->Insert(point);
        }
    }

    void Query(const AABB& queryBounds, std::vector<Vector2>* result)
    {
        if(CollisionAlgorithms::TestAABBCollision(&queryBounds, &boundary))
        {
            for(Vector2& point : points)
            {
                if(CollisionAlgorithms::TestPointAABBCollision(point, queryBounds))
                    result->push_back(point);
            }
        }

        if(divided)
        {
            topLeft->Query(queryBounds, result);
            topRight->Query(queryBounds, result);
            bottomLeft->Query(queryBounds, result);
            bottomRight->Query(queryBounds, result);
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

        topLeft = std::make_unique<QuadTree>(tl, this->capacity);
        topRight = std::make_unique<QuadTree>(tr, this->capacity);
        bottomLeft = std::make_unique<QuadTree>(bl, this->capacity);
        bottomRight = std::make_unique<QuadTree>(br, this->capacity);
    }

private:
    size_t capacity = 1;
    bool divided = false;
    AABB boundary;
    std::vector<Vector2> points;
    std::unique_ptr<QuadTree> topLeft = nullptr;
    std::unique_ptr<QuadTree> topRight = nullptr;
    std::unique_ptr<QuadTree> bottomLeft = nullptr;
    std::unique_ptr<QuadTree> bottomRight = nullptr;
};