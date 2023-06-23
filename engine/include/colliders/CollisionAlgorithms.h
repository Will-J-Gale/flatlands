#pragma once
#include <colliders/CollisionPoints.h>
#include <colliders/CircleCollider.h>
#include <colliders/LineCollider.h>
#include <colliders/BoxCollider.h>
#include <limits>
#include <Vector2.h>
#include <Transform.h>
#include <core/Logger.h>

namespace CollisionAlgorithms
{
    const static float FLOAT_MAX = std::numeric_limits<float>::max();
    const static float FLOAT_MIN = -FLOAT_MAX;

    struct AxisProjection
    {
        float min;
        float max;
    };

    inline AxisProjection projectShapeOntoAxis(Vector2 axis, std::vector<Vector2> shapeVertices)
    {
        AxisProjection projection;
        projection.min = FLOAT_MAX;
        projection.max = FLOAT_MIN;

        for(Vector2& vertex : shapeVertices)
        {
            float projectedPoint = Vector2::dot(axis, vertex);

            if(projectedPoint < projection.min)
                projection.min = projectedPoint;
                
            if(projectedPoint > projection.max)
                projection.max = projectedPoint;
        }

        return projection;
    }

    inline AxisProjection projectCircleOnToAxis(float radius, Vector2 position, Vector2 axis)
    {
        AxisProjection projection;

        Vector2 dir = axis.normalize() * radius;
        Vector2 point1 = position + dir;
        Vector2 point2 = position - dir;

        projection.min = Vector2::dot(point1, axis);
        projection.max = Vector2::dot(point2, axis);

        if(projection.min > projection.max)
        {
            float temp = projection.min;
            projection.min = projection.max;
            projection.max = temp;
        }

        return projection;
    }

    inline Vector2 getClosestVertexToPoint(Vector2 position, std::vector<Vector2> vertices)
    {
        float closestDistance = FLOAT_MAX;
        Vector2 closestPoint;

        for(Vector2 vertex : vertices)
        {
            float distance = Vector2::distance(position, vertex);
            if(distance < closestDistance)
            {
                closestDistance = distance;
                closestPoint = vertex;
            }
        }

        return closestPoint;
    }

    inline CollisionPoints FindCircleCircleCollisionPoints(
        CircleCollider* a, Transform* aTransform,
        CircleCollider* b, Transform* bTransform)
    {
        CollisionPoints collisionPoints;

        float distance = Vector2::distance(aTransform->position, bTransform->position);
        float radiusCombined = a->radius + b->radius;

        if(distance < radiusCombined)
        {
            //Penetration response
            Vector2 direction = aTransform->position - bTransform->position;
            float penetrationDepth = radiusCombined - direction.magnitude();
            Vector2 penetrationResolution = direction.normalize() * (penetrationDepth / 2.0f);

            collisionPoints.hasCollisions = true;
            collisionPoints.normal = direction.normalize();
            collisionPoints.depth = penetrationDepth;
        }

        return collisionPoints;
    }

    inline CollisionPoints FindLineCircleCollisionPoints(
        LineCollider* a, Transform* aTransform,
        CircleCollider* b, Transform* bTransform)
    {
        CollisionPoints collisionPoints;

        // Vector2 wallDirection = (a->end - a->start).normalize();
        
        // //Find closest point
        // Vector2 circleToLineStart = a->start - bTransform->position;
        // Vector2 circleToLineEnd = bTransform->position - a->end;
        // Vector2 closestPoint;

        // if(Vector2::dot(wallDirection, circleToLineStart) > 0)
        //     closestPoint = a->start;
        // else if(Vector2::dot(wallDirection, circleToLineEnd) > 0)
        //     closestPoint = a->end;
        // else
        // {
        //     float closestDist = Vector2::dot(wallDirection, circleToLineStart);
        //     Vector2 closestVector = wallDirection * closestDist;

        //     closestPoint = a->start - closestVector;
        // }

        // //Check if closest point is in circle
        // Vector2 penetrationVector = bTransform->position - closestPoint; 
        // float penetrationDepth = penetrationVector.magnitude();

        // if(penetrationDepth <= b->radius)
        // {
        //     collisionPoints.hasCollisions = true;
        //     collisionPoints.b = closestPoint;
        //     collisionPoints.a = bTransform->position - (penetrationVector.normalize() * b->radius);
        //     collisionPoints.normal = penetrationVector.normal();
        //     collisionPoints.depth = Vector2::distance(collisionPoints.a, collisionPoints.b);
        // }

        return collisionPoints;
    }

    inline CollisionPoints FindLineLineCollisionPoints(
        LineCollider* a, Transform* aTransform,
        LineCollider* b, Transform* bTransform)
    {
        CollisionPoints collisionPoints;
        collisionPoints.hasCollisions = true;
        AxisProjection projection1, projection2;

        auto aPoints = a->transformPoints(aTransform->position, aTransform->rotation);
        auto bPoints = b->transformPoints(bTransform->position, bTransform->rotation);

        auto aAxis = a->getAxis(aTransform->rotation).normal();
        auto bAxis = b->getAxis(bTransform->rotation).normal();

        //@TODO Duplicate code, refactorc
        projection1 = projectShapeOntoAxis(aAxis, aPoints);
        projection2 = projectShapeOntoAxis(aAxis, bPoints);
        float overlap = std::min(projection1.max, projection2.max) - std::max(projection1.min, projection2.min);

        if(overlap < 0)
        {
            collisionPoints.hasCollisions = false;
            return collisionPoints;
        }
        
        projection1 = projectShapeOntoAxis(bAxis, aPoints);
        projection2 = projectShapeOntoAxis(bAxis, bPoints);
        overlap = std::min(projection1.max, projection2.max) - std::max(projection1.min, projection2.min);

        if(overlap < 0)
        {
            collisionPoints.hasCollisions = false;
            return collisionPoints;
        }

        log("Collision");

        return collisionPoints;
    }

    inline CollisionPoints FindLineBoxCollisionPoints(
        LineCollider* a, Transform* aTransform,
        BoxCollider* b, Transform* bTransform)
    {
        CollisionPoints collisionPoints;

        return collisionPoints;
    }

    inline Vector2 getCenterPoint(std::vector<Vector2> vertices)
    {
        Vector2 center;

        for(Vector2 vertex : vertices)
        {
            center += vertex;
        }

        return center / vertices.size();
    }

    inline CollisionPoints FindBoxBoxCollision(
        BoxCollider* a, Transform* aTransform,
        BoxCollider* b, Transform* bTransform)
    {
        https://www.youtube.com/watch?v=SUyG3aV_vpM&list=PLSlpr6o9vURwq3oxVZSimY8iC-cdd3kIs&index=9

        //SAT 2.0
        //Algorithm is a little wonky because Y is flipped from most game engines (Normal up is positive, this down is positive)
        CollisionPoints collisionPoints;
        collisionPoints.depth = FLOAT_MAX;

        //Get shape vertices
        std::vector<Vector2> aPoints = a->transformPoints(aTransform->position, aTransform->rotation);
        std::vector<Vector2> bPoints = b->transformPoints(bTransform->position, bTransform->rotation);

        std::vector<Vector2> aAxes = a->getAxes(aTransform->rotation);
        std::vector<Vector2> bAxes = b->getAxes(bTransform->rotation);

        for(Vector2& axis : aAxes)
        {
            auto aProjection = projectShapeOntoAxis(axis, aPoints);
            auto bProjection = projectShapeOntoAxis(axis, bPoints);

            if(aProjection.min >= bProjection.max || bProjection.min >= aProjection.max)
            {
                collisionPoints.hasCollisions = false;
                return collisionPoints;
            }

            float axisDepth = std::min(bProjection.max - aProjection.min, aProjection.max - bProjection.min);

            if(axisDepth < collisionPoints.depth)
            {
                collisionPoints.depth = axisDepth;
                collisionPoints.normal = axis;
            }
        }

        for(Vector2& axis : bAxes)
        {
            auto aProjection = projectShapeOntoAxis(axis, aPoints);
            auto bProjection = projectShapeOntoAxis(axis, bPoints);

            if(aProjection.min >= bProjection.max || bProjection.min >= aProjection.max)
            {
                collisionPoints.hasCollisions = false;
                return collisionPoints;
            }

            float axisDepth = std::min(bProjection.max - aProjection.min, aProjection.max - bProjection.min);
            if(axisDepth < collisionPoints.depth)
            {
                collisionPoints.depth = axisDepth;
                collisionPoints.normal = axis;
            }
        }

        Vector2 centerA = getCenterPoint(aPoints);
        Vector2 centerB = getCenterPoint(bPoints);

        Vector2 dir = centerB - centerA;

        if(Vector2::dot(dir, collisionPoints.normal) < 0)
            collisionPoints.normal *= -1;

        collisionPoints.hasCollisions = true;
        return collisionPoints;
    }

    

    inline CollisionPoints FindBoxCircleCollision(
        BoxCollider* a, Transform* aTransform,
        CircleCollider* b, Transform* bTransform)
    {
        //SAT 2.0
        //Algorithm is a little wonky because Y is flipped from most game engines (Normal up is positive, this down is positive)
        CollisionPoints collisionPoints;
        collisionPoints.depth = FLOAT_MAX;

        //Get shape vertices
        std::vector<Vector2> aPoints = a->transformPoints(aTransform->position, aTransform->rotation);
        std::vector<Vector2> aAxes = a->getAxes(aTransform->rotation);

        for(Vector2& axis : aAxes)
        {
            auto aProjection = projectShapeOntoAxis(axis, aPoints);
            auto bProjection = projectCircleOnToAxis(b->radius, bTransform->position, axis);

            Vector2 box_projection = aTransform->position + (axis * aProjection.min);

            if(aProjection.min >= bProjection.max || bProjection.min >= aProjection.max)
            {
                collisionPoints.hasCollisions = false;
                return collisionPoints;
            }

            float axisDepth = std::min(bProjection.max - aProjection.min, aProjection.max - bProjection.min);

            if(axisDepth < collisionPoints.depth)
            {
                collisionPoints.depth = axisDepth;
                collisionPoints.normal = axis;
            }
        }

        Vector2 closestVertex = getClosestVertexToPoint(bTransform->position, aPoints);
        Vector2 axis = (closestVertex - bTransform->position).normalize();

        auto aProjection = projectShapeOntoAxis(axis, aPoints);
        auto bProjection = projectCircleOnToAxis(b->radius, bTransform->position, axis);

        if(aProjection.min >= bProjection.max || bProjection.min >= aProjection.max)
        {
            collisionPoints.hasCollisions = false;
            return collisionPoints;
        }

        float axisDepth = std::min(bProjection.max - aProjection.min, aProjection.max - bProjection.min);

        if(axisDepth < collisionPoints.depth)
        {
            collisionPoints.depth = axisDepth;
            collisionPoints.normal = axis;
        }

        Vector2 centerA = getCenterPoint(aPoints);
        Vector2 dir = bTransform->position - centerA;

        if(Vector2::dot(dir, collisionPoints.normal) < 0)
            collisionPoints.normal *= -1;

        collisionPoints.hasCollisions = true;
        return collisionPoints;
    }
}