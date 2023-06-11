#pragma once
#include <colliders/CollisionPoints.h>
#include <colliders/CircleCollider.h>
#include <colliders/LineCollider.h>
#include <colliders/BoxCollider.h>
#include <limits>
#include <Vector2.h>
#include <Transform.h>
#include <core/Logger.h>

namespace collisionAlgorithms
{
    static float FLOAT_MAX = std::numeric_limits<float>::max();

    struct AxisProjection
    {
        float min;
        float max;
        Vector2 collisionVertex;
    };

    inline AxisProjection projectShapeOntoAxis(Vector2 axis, std::vector<Vector2> shapeVertices)
    {
        AxisProjection projection;
        projection.min = Vector2::dot(axis, shapeVertices[0]);
        projection.max = projection.min;

        for(Vector2& vertex : shapeVertices)
        {
            float dot = Vector2::dot(axis, vertex);

            if(dot < projection.min)
            {
                projection.min = dot;
                projection.collisionVertex = vertex;
            }
                
            if(dot > projection.max)
                projection.max = dot;
        }

        return projection;
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
            collisionPoints.a = direction.normalize() * a->radius;
            collisionPoints.b = (direction.normalize() * -1) * b->radius;
            collisionPoints.normal = direction.normal();
            collisionPoints.depth = penetrationDepth;
        }

        return collisionPoints;
    }

    inline CollisionPoints FindLineCircleCollisionPoints(
        LineCollider* a, Transform* aTransform,
        CircleCollider* b, Transform* bTransform)
    {
        CollisionPoints collisionPoints;

        Vector2 wallDirection = (a->end - a->start).normalize();
        
        //Find closest point
        Vector2 circleToLineStart = a->start - bTransform->position;
        Vector2 circleToLineEnd = bTransform->position - a->end;
        Vector2 closestPoint;

        if(Vector2::dot(wallDirection, circleToLineStart) > 0)
            closestPoint = a->start;
        else if(Vector2::dot(wallDirection, circleToLineEnd) > 0)
            closestPoint = a->end;
        else
        {
            float closestDist = Vector2::dot(wallDirection, circleToLineStart);
            Vector2 closestVector = wallDirection * closestDist;

            closestPoint = a->start - closestVector;
        }

        //Check if closest point is in circle
        Vector2 penetrationVector = bTransform->position - closestPoint; 
        float penetrationDepth = penetrationVector.magnitude();

        if(penetrationDepth <= b->radius)
        {
            collisionPoints.hasCollisions = true;
            collisionPoints.b = closestPoint;
            collisionPoints.a = bTransform->position - (penetrationVector.normalize() * b->radius);
            collisionPoints.normal = penetrationVector.normal();
            collisionPoints.depth = Vector2::distance(collisionPoints.a, collisionPoints.b);
        }

        return collisionPoints;
    }

    inline CollisionPoints FindLineLineCollisionPoints(
        LineCollider* a, Transform* aTransform,
        LineCollider* b, Transform* bTransform)
    {
        CollisionPoints collisionPoints;

        return collisionPoints;
    }

    inline CollisionPoints FindLineBoxCollisionPoints(
        LineCollider* a, Transform* aTransform,
        BoxCollider* b, Transform* bTransform)
    {
        CollisionPoints collisionPoints;

        return collisionPoints;
    }

    inline CollisionPoints FindBoxBoxCollision(
        BoxCollider* a, Transform* aTransform,
        BoxCollider* b, Transform* bTransform)
    {
        //Separating axis theorum;
        //Collision resonse requires: Minimum translation vector

        float minOverlap = FLOAT_MAX;
        Vector2 smallestAxis;
        std::vector<Vector2> vertexObject;

        CollisionPoints collisionPoints;
        collisionPoints.hasCollisions = true;

        //Get shape vertices
        std::vector<Vector2> aPoints = a->getPoints(aTransform->position, aTransform->rotation);
        std::vector<Vector2> bPoints = a->getPoints(bTransform->position, bTransform->rotation);

        std::vector<Vector2> aAxes = a->getAxes(aTransform->rotation);
        std::vector<Vector2> bAxes = b->getAxes(bTransform->rotation);

        //@TODO Code duplication - refactor
        for(Vector2& axis : aAxes)
        {
            AxisProjection projection1 = projectShapeOntoAxis(axis, aPoints);
            AxisProjection projection2 = projectShapeOntoAxis(axis, bPoints);

            float overlap = std::min(projection1.max, projection2.max) - std::max(projection1.min, projection2.min);

            if(overlap < 0)
            {
                collisionPoints.hasCollisions = false;
                break;
            }

            if((projection1.max > projection2.max && projection1.min < projection2.min) ||
                projection1.max < projection2.max && projection1.min > projection2.min)
            {
                float min = std::abs(projection1.min - projection2.min);
                float max = std::abs(projection1.max - projection2.max);

                if(min < max)
                    overlap += min;
                else
                {
                    overlap += max;
                    axis = -axis;
                }


            }

            if(overlap < minOverlap)
            {
                minOverlap = overlap;
                smallestAxis = axis;
                vertexObject = bPoints;

                if(projection1.max > projection2.max)
                    smallestAxis = -axis;
            }

            
        }

        for(Vector2& axis : bAxes)
        {
            AxisProjection projection1 = projectShapeOntoAxis(axis, aPoints);
            AxisProjection projection2 = projectShapeOntoAxis(axis, bPoints);

            float overlap = std::min(projection1.max, projection2.max) - std::max(projection1.min, projection2.min);

            if(overlap < 0)
            {
                collisionPoints.hasCollisions = false;
                break;
            }

            if((projection1.max > projection2.max && projection1.min < projection2.min) ||
                projection1.max < projection2.max && projection1.min > projection2.min)
            {
                float min = std::abs(projection1.min - projection2.min);
                float max = std::abs(projection1.max - projection2.max);

                if(min < max)
                    overlap += min;
                else
                {
                    overlap += max;
                    axis = -axis;
                }


            }

            if(overlap < minOverlap)
            {
                minOverlap = overlap;
                smallestAxis = axis;
                vertexObject = aPoints;

                if(projection1.max < projection2.max)
                    smallestAxis = -axis;
            }
        }

        if(collisionPoints.hasCollisions)
        {
            //Video 16: 8:27
            log("HIT BOX!");
            Vector2 contactVertex = projectShapeOntoAxis(smallestAxis, vertexObject).collisionVertex;
            collisionPoints.normal = contactVertex.normal();
            collisionPoints.a = contactVertex;
            collisionPoints.b = contactVertex;

        }

        return collisionPoints;
    }

    inline CollisionPoints FindBoxCircleCollision(
        BoxCollider* a, Transform* aTransform,
        CircleCollider* b, Transform* bTransform)
    {
        CollisionPoints collisionPoints;

        return collisionPoints;
    }

    

    
}