#pragma once
#include <vector>

#include <collision/colliders/ConvexPolygonCollider.h>
#include <collision/CollisionPoints.h>
#include <math/Line.h>

class LineCollider : public ConvexPolygonCollider
{
public:
    LineCollider();
    LineCollider(Vector2 start, Vector2 end);
    LineCollider(Vector2 position, float width);

    Line getLine();
    Line transformLine(Transform* transform);

private:
    Line line;
};