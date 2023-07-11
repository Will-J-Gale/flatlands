#include <math/Line.h>
#include <core/Math.h>

Line::Line(Vector2 start, Vector2 end)
{
    this->start = start;
    this->end = end;
}

Vector2 Line::closestPointOnLine(Vector2 point)
{
    Vector2 lineDir = end - start;
    Vector2 lineStartToPoint = point - start;
    Vector2 closestPoint;

    float projection = Vector2::dot(lineStartToPoint, lineDir);
    float lineLength = lineDir.lengthSquared();
    float distance = projection / lineLength;

    if(distance <= 0)
        closestPoint = start;
    else if(distance >= 1)
        closestPoint = end;
    else
        closestPoint = start + (lineDir * distance);

    return closestPoint;
}

ClosestVertexProjection Line::closestVertexOnLine(std::vector<Vector2> vertices)
{
    ClosestVertexProjection closestVertexProjection;
    float closestDistance = Math::FLOAT_MAX;

    for(Vector2& vertex : vertices)
    {
        Vector2 vertexProjectedOnToLine = closestPointOnLine(vertex);
        float distanceToProjectedPoint = Vector2::distanceSquared(vertexProjectedOnToLine, vertex);

        if(distanceToProjectedPoint < closestDistance)
        {
            closestDistance = distanceToProjectedPoint;
            closestVertexProjection.projectedPoint = vertexProjectedOnToLine;
            closestVertexProjection.vertex = vertex;
        }
    }

    closestVertexProjection.distance = Vector2::distance(
        closestVertexProjection.projectedPoint, 
        closestVertexProjection.vertex
    );

    return closestVertexProjection;
}