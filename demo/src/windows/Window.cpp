#include <windows/Window.h>
#include <collision/CollisionAlgorithms.h>



ImVec2 Window::toImVec2(Vector2 v)
{
    return ImVec2(v.x, v.y);
}

void Window::DrawCapsule(ImDrawList *drawList, CapsuleCollider *capsule, Transform *transform, ImU32 colour)
{
    Vector2 position = transform->position;
    float radius = capsule->GetWidth() / 2.0f;
    float height = capsule->GetHeight();
    float halfHeight = height / 2.0f;
    Line centerLine = capsule->GetCenterLine(transform);

    drawList->AddCircleFilled(toImVec2(centerLine.start), radius, colour);
    drawList->AddCircleFilled(toImVec2(centerLine.end), radius, colour);

    float boxHeight = halfHeight - radius;

    drawList->AddQuadFilled(
        toImVec2(Vector2(-radius, -boxHeight).rotate(transform->rotation) + transform->position),
        toImVec2(Vector2(radius, -boxHeight).rotate(transform->rotation) + transform->position),
        toImVec2(Vector2(radius, boxHeight).rotate(transform->rotation) + transform->position),
        toImVec2(Vector2(-radius, boxHeight).rotate(transform->rotation) + transform->position),
        colour);
}

void Window::DrawBox(ImDrawList *drawList, BoxCollider *box, Transform *transform, ImU32 colour)
{
    auto points = box->transformPoints(transform);
    drawList->AddQuadFilled(
        toImVec2(points[0]),
        toImVec2(points[1]),
        toImVec2(points[2]),
        toImVec2(points[3]),
        colour);
}

void Window::DrawAABB(ImDrawList *drawList, AABB *aabb, ImU32 colour)
{
    float width = std::abs(aabb->max.x - aabb->min.x);
    float height = std::abs(aabb->max.y - aabb->min.y);
    drawList->AddQuad(
        toImVec2(aabb->min),
        toImVec2(aabb->min + Vector2(width, 0)),
        toImVec2(aabb->max),
        toImVec2(aabb->min + Vector2(0, height)),
        GREEN);
}

void Window::DrawPolygon(ImDrawList* drawList, ConvexPolygonCollider* polygon, Transform* transform, ImU32 colour)
{
    std::vector<Vector2> vertices = polygon->transformPoints(transform);
    std::vector<ImVec2> pointsToDraw;

    for (size_t i = 0; i < vertices.size(); i++)
    {
        pointsToDraw.push_back(std::move(toImVec2(vertices[i])));
    };

    drawList->AddConvexPolyFilled(&pointsToDraw[0], vertices.size(), colour);
}

void Window::DrawCircleOnAxis(ImDrawList *drawList, Vector2 axis, float radius, Vector2 position)
{
    auto circleProjection = CollisionAlgorithms::projectCircleOnToAxis(radius, position, axis);
    drawList->AddCircleFilled(toImVec2(axis * circleProjection.min), 4.0f, RED);
    drawList->AddCircleFilled(toImVec2(axis * circleProjection.max), 7.0f, RED);
}

void Window::DrawPolygonOnAxis(ImDrawList *drawList, Vector2 axis, std::vector<Vector2> vertices)
{
    auto polygonProjection = CollisionAlgorithms::projectShapeOntoAxis(axis, vertices);
    drawList->AddCircleFilled(toImVec2(axis * polygonProjection.min), 4.0f, GREEN);
    drawList->AddCircleFilled(toImVec2(axis * polygonProjection.max), 7.0f, GREEN);
}
void Window::DrawAxis(ImDrawList *drawList, Vector2 axis, float length)
{
    Vector2 positiveEnd = axis * length;
    Vector2 negativeEnd = -axis * length;
    drawList->AddLine(toImVec2(Vector2()), toImVec2(positiveEnd), BLUE, 2.0f);
    drawList->AddLine(toImVec2(Vector2()), toImVec2(negativeEnd), WHITE, 2.0f);
}