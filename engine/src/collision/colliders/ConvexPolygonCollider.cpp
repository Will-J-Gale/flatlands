#include <algorithm>
#include <set>

#include <collision/colliders/ConvexPolygonCollider.h>
#include <core/Math.h>
#include <core/Logger.h>

inline auto xTest = [](Vector2 a, Vector2 b){
    return a.x < b.x;
};

ConvexPolygonCollider::ConvexPolygonCollider(std::vector<Vector2> points)
{
    this->setVertices(points);
}

void ConvexPolygonCollider::setVertices(std::vector<Vector2> vertices)
{
    // Compute convex hull
    std::vector<Vector2> convexHull;
    std::vector<Vector2> sortedVertices = vertices;
    std::sort(
        sortedVertices.begin(), 
        sortedVertices.end(), 
        xTest
    );

    Vector2 leftMost = sortedVertices[0];
    convexHull.push_back(leftMost);

    size_t i = 0;
    size_t sanity_check = 0;

    while(true)
    {
        Vector2 pointOnHull = convexHull[i];
        Vector2 currentBestHullMatch = vertices[0];

        //Find left most vertex in vertices from pointOnHull
        for(Vector2& vertex : vertices)
        {
            Vector2 a = currentBestHullMatch - pointOnHull;
            Vector2 b  = vertex - pointOnHull;

            float cross = Vector2::cross(a, b);

            if(currentBestHullMatch == pointOnHull || cross < 0)
                currentBestHullMatch = vertex;
        }


        pointOnHull = currentBestHullMatch;
        if(pointOnHull == leftMost)
            break;
        i++;
        convexHull.push_back(pointOnHull);
        sanity_check++;


        if(sanity_check >= 100000)
            throw std::runtime_error("INFINITE LOOP!");
    }

    Vector2 center = Vector2::findCenterPoint(vertices);
    for(Vector2& vertex : convexHull)
    {
        this->vertices.push_back(vertex - center);
    }
}

std::vector<Vector2> ConvexPolygonCollider::transformPoints(Transform* transform)
{
    std::vector<Vector2> transformedPoints;

    for(Vector2& vertex : this->vertices)
    {
        transformedPoints.push_back(vertex.rotate(transform->rotation) + transform->position);
    }

    return transformedPoints;
}

std::vector<Vector2> ConvexPolygonCollider::getAxes(float radians)
{
    std::vector<Vector2> axes;
    std::set<std::string> existingAxes;

    for(int i = 0; i < vertices.size(); i++)
    {
        Vector2 start = vertices[i].rotate(radians);
        Vector2 end = vertices[(i + 1) % vertices.size()].rotate(radians);

        Vector2 axis = (end - start).normal();
        //Hacky way to check for duplicate axes
        std::string vectorHash = std::to_string(axis.x) + std::to_string(axis.y);
        std::string negativeVectorHash = std::to_string(-axis.x) + std::to_string(-axis.y);

        if(existingAxes.count(vectorHash) == 0 && existingAxes.count(negativeVectorHash) == 0)
        {
            existingAxes.insert(vectorHash);
            existingAxes.insert(negativeVectorHash);
            axes.push_back(axis);
        }
    }

    return axes;
}

std::vector<Line> ConvexPolygonCollider::getEdges(Transform* transform)
{
    std::vector<Vector2> transformedVertices = transformPoints(transform);
    std::vector<Line> edges;

    for(int i = 0; i < transformedVertices.size(); i++)
    {
        Vector2 v1 = transformedVertices[i];
        Vector2 v2 = transformedVertices[(i + 1) % vertices.size()];

        edges.push_back(Line(v2, v1));
    }

    return edges;
}

AABB ConvexPolygonCollider::GetAABB(Transform* transform)
{
    Vector2 min = Vector2(Math::FLOAT_MAX, Math::FLOAT_MAX);
    Vector2 max = Vector2(Math::FLOAT_MIN, Math::FLOAT_MIN);

    std::vector<Vector2> transformedPoints = transformPoints(transform);

    for(Vector2& point : transformedPoints)
    {
        if(point.x < min.x)
            min.x = point.x;

        if(point.x > max.x)
            max.x = point.x;

        if(point.y < min.y)
            min.y = point.y;

        if(point.y > max.y)
            max.y = point.y;
    }

    return AABB(min, max);
};

float ConvexPolygonCollider::GetRotationalInertia(float mass)
{
    //@TODO - This is a copy of the circle
    //For correct moment of inertia https://en.wikipedia.org/wiki/List_of_moments_of_inertia
    return 0.5f * mass * (25 * 25);
}