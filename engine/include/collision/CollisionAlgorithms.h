#pragma once
#include <collision/CollisionPoints.h>
#include <collision/colliders/CircleCollider.h>
#include <collision/colliders/LineCollider.h>
#include <collision/colliders/ConvexPolygonCollider.h>
#include <collision/colliders/CapsuleCollider.h>
#include <collision/Collision.h>
#include <Vector2.h>
#include <Line.h>
#include <Transform.h>
#include <core/Logger.h>
#include <core/Math.h>
#include <core/Core.h>

namespace CollisionAlgorithms
{
    struct AxisProjection
    {
        float min;
        float max;
    };

    inline AxisProjection projectShapeOntoAxis(Vector2 axis, std::vector<Vector2> shapeVertices)
    {
        AxisProjection projection;
        projection.min = Math::FLOAT_MAX;
        projection.max = Math::FLOAT_MIN;

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
        float closestDistance = Math::FLOAT_MAX;
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

    inline bool TestAABBCollision(AABBCollider* a, AABBCollider* b)
    {
        return a->max.x > b->min.x &&
			a->min.x < b->max.x &&
			a->max.y > b->min.y &&
			a->min.y < b->max.y;
    }

    inline CollisionPoints TestCircleCircleCollision(
        CircleCollider* a, Transform* aTransform,
        CircleCollider* b, Transform* bTransform)
    {
        CollisionPoints collisionPoints;

        float distance = Vector2::distance(aTransform->position, bTransform->position);
        float radiusCombined = a->radius + b->radius;

        if(distance < radiusCombined)
        {
            Vector2 direction = bTransform->position - aTransform->position;
            float penetrationDepth = radiusCombined - direction.magnitude();

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

        auto aPoints = a->transformPoints(aTransform);
        auto bPoints = b->transformPoints(bTransform);

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

        return collisionPoints;
    }

    inline CollisionPoints FindLineBoxCollisionPoints(
        LineCollider* a, Transform* aTransform,
        ConvexPolygonCollider* b, Transform* bTransform)
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

    inline CollisionPoints TestBoxBoxCollision(
        ConvexPolygonCollider* a, Transform* aTransform,
        ConvexPolygonCollider* b, Transform* bTransform)
    {
        https://www.youtube.com/watch?v=SUyG3aV_vpM&list=PLSlpr6o9vURwq3oxVZSimY8iC-cdd3kIs&index=9

        //SAT 2.0
        //Algorithm is a little wonky because Y is flipped from most game engines (Normal up is positive, this down is positive)
        CollisionPoints collisionPoints;
        collisionPoints.depth = Math::FLOAT_MAX;

        //Get shape vertices
        std::vector<Vector2> aPoints = a->transformPoints(aTransform);
        std::vector<Vector2> bPoints = b->transformPoints(bTransform);

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

    inline CollisionPoints TestBoxCircleCollision(
        ConvexPolygonCollider* a, Transform* aTransform,
        CircleCollider* b, Transform* bTransform)
    {
        //SAT 2.0
        //Algorithm is a little wonky because Y is flipped from most game engines (Normal up is positive, this down is positive)
        CollisionPoints collisionPoints;
        collisionPoints.depth = Math::FLOAT_MAX;

        //Get shape vertices
        std::vector<Vector2> aPoints = a->transformPoints(aTransform);
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

        Vector2 dir = bTransform->position - aTransform->position;

        if(Vector2::dot(dir, collisionPoints.normal) < 0)
            collisionPoints.normal *= -1;

        collisionPoints.hasCollisions = true;
        return collisionPoints;
    }

    inline CollisionPoints TestCircleCapsuleCollision(
        CircleCollider* a, Transform* aTransform,
        CapsuleCollider* b, Transform* bTransform)
    {
        CollisionPoints collisionPoints;

        float capsuleRadius = b->GetWidth() / 2.0f;
        Line capsuleCenterLine = b->GetCenterLine(bTransform);
        Vector2 closestPoint = capsuleCenterLine.closestPointOnLine(aTransform->position);
        float distanceToCircle = Vector2::distance(closestPoint, aTransform->position);
        float radiusSum = capsuleRadius + a->radius;

        if(distanceToCircle <= radiusSum)
        {
            Vector2 direction = closestPoint - aTransform->position;
            float depth = radiusSum - direction.magnitude();

            collisionPoints.normal = direction.normalize();
            collisionPoints.depth = depth;
            collisionPoints.hasCollisions = true;
        } 

        return collisionPoints;
    }

    inline CollisionPoints TestCapsuleCapsuleCollision(
        CapsuleCollider* a, Transform* aTransform,
        CapsuleCollider* b, Transform* bTransform
    )
    {
        CollisionPoints collisionPoints;
        Line aLine = a->GetCenterLine(aTransform);
        Line bLine = a->GetCenterLine(bTransform);

        ClosestVertexProjection closestVertexProjectionOnA = aLine.closestVertexOnLine({bLine.start, bLine.end});
        ClosestVertexProjection closestVertexProjectionOnB = bLine.closestVertexOnLine({aLine.start, aLine.end});
        ClosestVertexProjection closestVertexProjection = closestVertexProjectionOnA;

        if(closestVertexProjectionOnB.distance < closestVertexProjectionOnA.distance)
            closestVertexProjection = closestVertexProjectionOnB;

        float aRadius = a->GetWidth() / 2.0f;
        float bRadius = b->GetWidth() / 2.0f;
        float radiusSum = aRadius + bRadius;

        Vector2 bodyDir = bTransform->position - aTransform->position;

        if(closestVertexProjection.distance < radiusSum)
        {
            Vector2 normal = (closestVertexProjection.projectedPoint - closestVertexProjection.vertex).normalize();
            if(Vector2::dot(normal, bodyDir) < 0)
                normal *= -1;

            collisionPoints.normal = normal;
            collisionPoints.depth = radiusSum - closestVertexProjection.distance;
            collisionPoints.hasCollisions = true;
        }

        return collisionPoints;
    }

    inline CollisionPoints TestCollision(
        Collider* aCollider, Transform* aTransform,
        Collider* bCollider, Transform* bTransform)
    {
        ColliderType aType = aCollider->GetType();
        ColliderType bType = bCollider->GetType();

        if(aType == ColliderType::POLYGON)
        {
            if(bType == ColliderType::POLYGON)
            {
                return TestBoxBoxCollision(
                    dynamic_cast<ConvexPolygonCollider*>(aCollider), aTransform, 
                    dynamic_cast<ConvexPolygonCollider*>(bCollider), bTransform
                );
            }
            else if(bType == ColliderType::CIRCLE)
            {
                return TestBoxCircleCollision(
                    dynamic_cast<ConvexPolygonCollider*>(aCollider), aTransform, 
                    dynamic_cast<CircleCollider*>(bCollider), bTransform
                );
            }
            else
            {
                return CollisionPoints();
            }
        }
        else if(aType == ColliderType::CIRCLE)
        {
            if(bType == ColliderType::POLYGON)
            {
                CollisionPoints collisionPoints = TestBoxCircleCollision(
                    dynamic_cast<ConvexPolygonCollider*>(bCollider), bTransform, 
                    dynamic_cast<CircleCollider*>(aCollider), aTransform
                );

                //Flip normal becuase Collision test flipped A and B
                collisionPoints.normal *= -1;
                return collisionPoints;
            }
            else if(bType == ColliderType::CIRCLE)
            {
                return TestCircleCircleCollision(
                    dynamic_cast<CircleCollider*>(aCollider), aTransform, 
                    dynamic_cast<CircleCollider*>(bCollider), bTransform
                );
            }
            else if(bType == ColliderType::CAPSULE)
            {
                return TestCircleCapsuleCollision(
                    dynamic_cast<CircleCollider*>(aCollider), aTransform, 
                    dynamic_cast<CapsuleCollider*>(bCollider), bTransform
                );
            }
            else
            {
                return CollisionPoints();
            }
        }
        else if(aType == ColliderType::CAPSULE)
        {
            if(bType == ColliderType::CIRCLE)
            {
                CollisionPoints collisionPoints = TestCircleCapsuleCollision(
                    dynamic_cast<CircleCollider*>(bCollider), bTransform, 
                    dynamic_cast<CapsuleCollider*>(aCollider), aTransform
                );

                collisionPoints.normal *= -1;
                return collisionPoints;

            }
            else if(bType == ColliderType::CAPSULE)
            {
                return TestCapsuleCapsuleCollision(
                    dynamic_cast<CapsuleCollider*>(aCollider), aTransform, 
                    dynamic_cast<CapsuleCollider*>(bCollider), bTransform
                );

            }
            else
            {
                return CollisionPoints();
            }
        }
        else
        {
            //Line
            return CollisionPoints();
        }
    }

    inline void GenerateBoxBoxContactPoints(
        ConvexPolygonCollider* aCollider, Transform* aTransform,
        ConvexPolygonCollider* bCollider, Transform* bTransform, 
        CollisionPoints* collisionPoints)
    {
        std::vector<Vector2> aPoints = aCollider->transformPoints(aTransform);
        std::vector<Vector2> bPoints = bCollider->transformPoints(bTransform);
        std::vector<Line> aEdges = aCollider->getEdges(aTransform);
        std::vector<Line> bEdges = bCollider->getEdges(bTransform);

        int contactCount = 0;
        float closestDistance = Math::FLOAT_MAX;
        Vector2 contact1, contact2;

        for(Vector2& vertex : aPoints)
        {
            for(Line edge : bEdges)
            {
                Vector2 pointOnLine = edge.closestPointOnLine(vertex);                
                float distance = Vector2::distanceSquared(pointOnLine, vertex);

                if(Math::nearlyEqual(distance, closestDistance))
                {
                    if(!Vector2::nearlyEqual(pointOnLine, contact1))
                    {
                        contact2 = pointOnLine;
                        contactCount = 2;
                    }
                }
                else if(distance < closestDistance)
                {
                    closestDistance = distance;
                    contact1 = pointOnLine;
                    contactCount = 1;
                }
            }
        }

        for(Vector2& vertex : bPoints)
        {
            for(Line edge : aEdges)
            {
                Vector2 pointOnLine = edge.closestPointOnLine(vertex);                
                float distance = Vector2::distanceSquared(pointOnLine, vertex);

                if(Math::nearlyEqual(distance, closestDistance))
                {
                    if(!Vector2::nearlyEqual(pointOnLine, contact1))
                    {
                        contact2 = pointOnLine;
                        contactCount = 2;
                    }
                }
                else if(distance < closestDistance)
                {
                    closestDistance = distance;
                    contact1 = pointOnLine;
                    contactCount = 1;
                }
            }
        }

        collisionPoints->contacts.emplace_back(std::move(contact1));

        if(contactCount == 2)
            collisionPoints->contacts.emplace_back(std::move(contact2));
    }

    inline void GenerateCircleCircleContactPoints(
        CircleCollider* aCollider, Transform* aTransform,
        CircleCollider* bCollider, Transform* bTransform,
        CollisionPoints* collisionPoints)
    {
        Vector2 dir = (bTransform->position - aTransform->position).normalize();
        Vector2 contact = aTransform->position + (dir * aCollider->radius);
        collisionPoints->contacts.emplace_back(std::move(contact));
    }

    inline void GenerateLineLineContactPoints(
        LineCollider* aCollider, Transform* aTransform, 
        LineCollider* bCollider, Transform* bTransform, 
        CollisionPoints* collisionPoints)
    {
        
    }

    inline void GenerateCircleBoxContactPoints(
        CircleCollider* aCollider, Transform* aTransform, 
        ConvexPolygonCollider* bCollider, Transform* bTransform, 
        CollisionPoints* collisionPoints)
    {
        Vector2 contactPoint;
        float closestDistance = Math::FLOAT_MAX;

        for(Line& edge : bCollider->getEdges(bTransform))
        {
            Vector2 pointOnLine = edge.closestPointOnLine(aTransform->position);
            float distanceSquared = Vector2::distanceSquared(pointOnLine, aTransform->position);

            if(distanceSquared < closestDistance)
            {
                closestDistance = distanceSquared;
                contactPoint = pointOnLine;
            }
        }

        collisionPoints->contacts.emplace_back(std::move(contactPoint));
    }

    inline void GenerateCircleLineContactPoints(
        CircleCollider* aCollider, Transform* aTransform, 
        LineCollider* bCollider, Transform* bTransform,
        CollisionPoints* collisionPoints)
    {
        
    }

    inline void GenerateBoxLineContactPoints(
        ConvexPolygonCollider* aCollider, Transform* aTransform,
        LineCollider* bCollider, Transform* bTransform,
        CollisionPoints* collisionPoints)
    {
        
    }

    inline void GenerateCircleCapsuleContactPoints(
        CircleCollider* a, Transform* aTransform,
        CapsuleCollider* b, Transform* bTransform,
        CollisionPoints* collisionPoints
    )
    {
        Vector2 contact = aTransform->position + (-collisionPoints->normal * a->radius);
        collisionPoints->contacts.push_back(contact);
    }

    inline void GenerateCapsuleCapsuleContactPoints(
        CapsuleCollider* aCollider, Transform* aTransform,
        CapsuleCollider* bCollider, Transform* bTransform,
        CollisionPoints* collisionPoints)
    {
        // Vector2 dir = (bTransform->position - aTransform->position).normalize();
        // Vector2 contact = aTransform->position + (dir * aCollider->radius);
        // collisionPoints->contacts.emplace_back(std::move(contact));

        float aRadius = aCollider->GetWidth() / 2.0f;
        Line aLine = aCollider->GetCenterLine(aTransform);
        Line bLine = bCollider->GetCenterLine(bTransform);
        ClosestVertexProjection closestVertexProjection = aLine.closestVertexOnLine({bLine.start, bLine.end});
        Vector2 contactPoint = closestVertexProjection.projectedPoint + (collisionPoints->normal * aRadius);

        collisionPoints->contacts.push_back(std::move(contactPoint));
    }

    inline void GenerateContactPoints(Collision& collision)
    {
        ColliderType aType = collision.a->collider->GetType();
        ColliderType bType = collision.b->collider->GetType();
        Transform* aTransform = &collision.a->transform;
        Transform* bTransform = &collision.b->transform;
        Collider* aCollider = collision.a->collider;
        Collider* bCollider = collision.b->collider;

        if(aType == ColliderType::POLYGON)
        {
            if(bType == ColliderType::POLYGON)
            {
                GenerateBoxBoxContactPoints(
                    static_cast<ConvexPolygonCollider*>(aCollider), aTransform,
                    static_cast<ConvexPolygonCollider*>(bCollider), bTransform,
                    &collision.collisionPoints
                );
            }
            else if(bType == ColliderType::CIRCLE)
            {
                GenerateCircleBoxContactPoints(
                    static_cast<CircleCollider*>(bCollider), bTransform,
                    static_cast<ConvexPolygonCollider*>(aCollider), aTransform,
                    &collision.collisionPoints
                );
            }
            else
            {
                //@TODO 
            }
        }
        else if(aType == ColliderType::CIRCLE)
        {
            if(bType == ColliderType::POLYGON)
            {
                GenerateCircleBoxContactPoints(
                    static_cast<CircleCollider*>(aCollider), aTransform,
                    static_cast<ConvexPolygonCollider*>(bCollider), bTransform,
                    &collision.collisionPoints
                ); 
            }
            else if(bType == ColliderType::CIRCLE)
            {
                GenerateCircleCircleContactPoints(
                    static_cast<CircleCollider*>(aCollider), aTransform,
                    static_cast<CircleCollider*>(bCollider), bTransform,
                    &collision.collisionPoints
                );
            }
            else if(bType == ColliderType::CAPSULE)
            {
                GenerateCircleCapsuleContactPoints(
                    static_cast<CircleCollider*>(aCollider), aTransform,
                    static_cast<CapsuleCollider*>(bCollider), bTransform,
                    &collision.collisionPoints
                );
            }
            else
            {
                //@TODO 
            }
        }
        else if(aType == ColliderType::CAPSULE)
        {
            if(bType == ColliderType::CIRCLE)
            {
                GenerateCircleCapsuleContactPoints(
                    static_cast<CircleCollider*>(bCollider), bTransform,
                    static_cast<CapsuleCollider*>(aCollider), aTransform,
                    &collision.collisionPoints
                );
            }
            if(bType == ColliderType::CAPSULE)
            {
                GenerateCapsuleCapsuleContactPoints(
                    static_cast<CapsuleCollider*>(aCollider), aTransform,
                    static_cast<CapsuleCollider*>(bCollider), bTransform,
                    &collision.collisionPoints
                );
            }
        }
        else
        {
            //@TODO 
        }
    }
}