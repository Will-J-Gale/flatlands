#include <windows/collisionDetection/CircleCapsuleCollisionDetectionWindow.h>

CircleCapsuleCollisionDetectionWindow::CircleCapsuleCollisionDetectionWindow()
{
    a = CircleCollider(50.0f);
    b = CapsuleCollider(100, 200);

    aTransform.position = Vector2(500, 500);
}

void CircleCapsuleCollisionDetectionWindow::Render()
{
    ImGui::Begin("Circle-Capsule Collision");
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    Vector2 mousePosition = Vector2(ImGui::GetMousePos().x, ImGui::GetMousePos().y);
    bTransform.position = mousePosition;

    float rotationAmount = 0.05f;

    if(ImGui::IsKeyDown(ImGuiKey_Q))
        bTransform.rotation -= rotationAmount;
    else if(ImGui::IsKeyDown(ImGuiKey_E))
        bTransform.rotation += rotationAmount;

    drawList->AddCircleFilled(toImVec2(aTransform.position), a.radius, WHITE);
    DrawCapsule(drawList, &b, &bTransform, WHITE);

    float capsuleRadius = b.GetWidth() / 2.0f;
    Line capsuleCenterLine = b.GetCenterLine(&bTransform);
    Vector2 closestPoint = capsuleCenterLine.closestPointOnLine(aTransform.position);
    float distanceToCircle = Vector2::distance(closestPoint, aTransform.position);
    float radiusSum = capsuleRadius + a.radius;

    drawList->AddCircle(toImVec2(closestPoint), capsuleRadius, RED);

    if (distanceToCircle <= radiusSum)
    {
        Vector2 direction = aTransform.position - closestPoint;
        Vector2 contact = aTransform.position + (-direction.normalize() * a.radius);
        float depth = radiusSum - direction.magnitude();

        // Vector2 normal = direction.normalize();
        Vector2 end = contact + (direction.normalize() * depth);
        drawList->AddLine(toImVec2(contact), toImVec2(end), RED, 2.0f);
        drawList->AddCircleFilled(toImVec2(contact), 3.0f, RED);

        Transform newTransform = bTransform;
        newTransform.position += (-direction.normalize() * depth);
        DrawCapsule(drawList, &b, &newTransform, MAGENTA);
    }

    drawList->AddLine(toImVec2(aTransform.position), toImVec2(closestPoint), GREEN);
    drawList->AddCircleFilled(toImVec2(closestPoint), 5.0f, CYAN);

    ImGui::End();
}