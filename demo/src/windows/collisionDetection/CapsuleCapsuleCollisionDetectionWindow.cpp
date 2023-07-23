#include <windows/collisionDetection/CapsuleCapsuleCollisionDetectionWindow.h>

CapsuleCapsuleCollisionDetectionWindow::CapsuleCapsuleCollisionDetectionWindow()
{
    a = CapsuleCollider(50, 200);
    b = CapsuleCollider(100, 300);
    bTransform.position = Vector2(500, 500);
}

void CapsuleCapsuleCollisionDetectionWindow::Render()
{
    ImGui::Begin("Capsule-Capsule Collision");
    Vector2 mousePosition(ImGui::GetMousePos().x, ImGui::GetMousePos().y);
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    aTransform.position = mousePosition;

    float rotationAmount = 0.05;

    if(ImGui::IsKeyDown(ImGuiKey::ImGuiKey_Q))
        aTransform.rotation -= rotationAmount;
    else if(ImGui::IsKeyDown(ImGuiKey::ImGuiKey_E))
        aTransform.rotation += rotationAmount;

    DrawCapsule(drawList, &a, &aTransform, WHITE);
    DrawCapsule(drawList, &b, &bTransform, WHITE);

    Line aLine = a.GetCenterLine(&aTransform);
    Line bLine = b.GetCenterLine(&bTransform);

    drawList->AddLine(toImVec2(aLine.start), toImVec2(aLine.end), BLACK);
    drawList->AddLine(toImVec2(bLine.start), toImVec2(bLine.end), BLACK);

    ClosestVertexProjection closestVertexProjectionOnA = aLine.closestVertexOnLine({bLine.start, bLine.end});
    ClosestVertexProjection closestVertexProjectionOnB = bLine.closestVertexOnLine({aLine.start, aLine.end});
    ClosestVertexProjection closestProjection = closestVertexProjectionOnA;
    bool vertexIsA = false;

    if (closestVertexProjectionOnB.distance < closestVertexProjectionOnA.distance)
    {
        closestProjection = closestVertexProjectionOnB;
        vertexIsA = true;
    }

    drawList->AddCircleFilled(toImVec2(closestProjection.vertex), 3.0f, MAGENTA);
    drawList->AddLine(toImVec2(closestProjection.projectedPoint), toImVec2(closestProjection.vertex), MAGENTA, 2.0f);

    float aRadius = a.GetWidth() / 2.0f;
    float bRadius = b.GetWidth() / 2.0f;
    float radiusSum = aRadius + bRadius;

    if (vertexIsA)
    {
        drawList->AddCircle(toImVec2(closestProjection.vertex), aRadius, RED);
        drawList->AddCircle(toImVec2(closestProjection.projectedPoint), bRadius, RED);
    }
    else
    {
        drawList->AddCircle(toImVec2(closestProjection.vertex), bRadius, RED);
        drawList->AddCircle(toImVec2(closestProjection.projectedPoint), aRadius, RED);
    }

    if (closestProjection.distance < radiusSum)
    {
        Vector2 normal = (closestProjection.projectedPoint - closestProjection.vertex).normalize();

        if(!vertexIsA)
            normal *= -1;

        float depth = radiusSum - closestProjection.distance;
        Transform resolvedCapsuleTransform = aTransform;
        resolvedCapsuleTransform.position -= normal * depth;

        DrawCapsule(drawList, &a, &resolvedCapsuleTransform, BLUE);

        float radiusOfVertex = vertexIsA ? aRadius : bRadius;
        Vector2 dir = (closestProjection.projectedPoint - closestProjection.vertex).normalize();
        Vector2 contactPoint = closestProjection.vertex + (dir * radiusOfVertex);

        drawList->AddCircleFilled(toImVec2(contactPoint), 5.0f, GREEN);
    }

    ImGui::End();
}