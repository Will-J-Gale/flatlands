#include <windows/collisionDetection/CirclePolygonCollisionDetectionWindow.h>
#include <collision/CollisionAlgorithms.h>

CirclePolygonCollisionDetectionWindow::CirclePolygonCollisionDetectionWindow()
{
    a = CircleCollider(100.0f);
    b = BoxCollider(200, 200);
    aTransform.position = Vector2(100, 100);
}

ImVec2 CirclePolygonCollisionDetectionWindow::toImVec2(Vector2 v)
{
    return ImVec2(
        v.x + windowOffset.x,
        v.y + windowOffset.y
    );
}
void CirclePolygonCollisionDetectionWindow::Render()
{
    ImGui::Begin("Circle-Polygon Collision");
    Vector2 mousePosition(ImGui::GetMousePos().x, ImGui::GetMousePos().y);
    mousePosition -= windowOffset;
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    bTransform.position = mousePosition;

    DrawBox(drawList, &b, &bTransform, WHITE);
    drawList->AddCircleFilled(toImVec2(aTransform.position), a.radius, WHITE);

    float rotationAmount = 0.05;

    if(ImGui::IsKeyDown(ImGuiKey::ImGuiKey_Q))
        bTransform.rotation -= rotationAmount;
    else if(ImGui::IsKeyDown(ImGuiKey::ImGuiKey_E))
        bTransform.rotation += rotationAmount;

    //SAT 2.0
    //Algorithm is a little wonky because Y is flipped from most game engines (Normal up is positive, this down is positive)
    CollisionPoints collisionPoints;
    collisionPoints.hasCollisions = true;
    collisionPoints.depth = Math::FLOAT_MAX;

    //Get shape vertices
    std::vector<Vector2> bPoints = b.transformPoints(&bTransform);
    std::vector<Vector2> bAxes = b.getAxes(bTransform.rotation);

    for(Vector2& axis : bAxes)
    {
        auto aProjection = CollisionAlgorithms::projectShapeOntoAxis(axis, bPoints);
        auto bProjection = CollisionAlgorithms::projectCircleOnToAxis(a.radius, aTransform.position, axis);

        Vector2 box_projection = bTransform.position + (axis * aProjection.min);

        if(aProjection.min >= bProjection.max || bProjection.min >= aProjection.max)
            collisionPoints.hasCollisions = false;

        DrawAxis(drawList, axis);
        DrawPolygonOnAxis(drawList, axis, bPoints);
        DrawCircleOnAxis(drawList, axis, a.radius, aTransform.position);

        float axisDepth = std::min(bProjection.max - aProjection.min, aProjection.max - bProjection.min);

        if(axisDepth < collisionPoints.depth)
        {
            collisionPoints.depth = axisDepth;
            collisionPoints.normal = axis;
        }
    }

    Vector2 closestVertex = CollisionAlgorithms::getClosestVertexToPoint(aTransform.position, bPoints);
    Vector2 axis = (closestVertex - aTransform.position).normalize();

    drawList->AddCircleFilled(toImVec2(closestVertex), 10.0f, CYAN);
    auto aProjection = CollisionAlgorithms::projectShapeOntoAxis(axis, bPoints);
    auto bProjection = CollisionAlgorithms::projectCircleOnToAxis(a.radius, aTransform.position, axis);

    DrawAxis(drawList, axis);
    DrawPolygonOnAxis(drawList, axis, bPoints);
    DrawCircleOnAxis(drawList, axis, a.radius, aTransform.position);
    drawList->AddLine(toImVec2(closestVertex), toImVec2(aTransform.position), GREEN);

    if(aProjection.min >= bProjection.max || bProjection.min >= aProjection.max)
        collisionPoints.hasCollisions = false;

    float axisDepth = std::min(bProjection.max - aProjection.min, aProjection.max - bProjection.min);

    if(axisDepth < collisionPoints.depth)
    {
        collisionPoints.depth = axisDepth;
        collisionPoints.normal = axis;

        
    }

    if(collisionPoints.hasCollisions)
    {
        Vector2 dir = aTransform.position - bTransform.position;

        if(Vector2::dot(dir, collisionPoints.normal) < 0)
            collisionPoints.normal *= -1;

        Transform resolvedCollisionTranform = bTransform;
        resolvedCollisionTranform.position -= (collisionPoints.normal * collisionPoints.depth);

        DrawBox(drawList, &b, &resolvedCollisionTranform, CYAN_A);
        Vector2 normalEnd = bTransform.position + (collisionPoints.normal * collisionPoints.depth);
        drawList->AddLine(toImVec2(bTransform.position), toImVec2(normalEnd), GREEN, 2.0f);
    }

    ImGui::End();
}