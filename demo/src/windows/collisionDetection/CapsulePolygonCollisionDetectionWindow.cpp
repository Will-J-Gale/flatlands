#include <windows/collisionDetection/CapsulePolygonCollisionDetectionWindow.h>
#include <collision/CollisionAlgorithms.h>
#include <collision/CollisionPoints.h>

CapsulePolygonCollisionDetectionWindow::CapsulePolygonCollisionDetectionWindow()
{
    a = CapsuleCollider(100, 200);


    float radius = 100;
    float angle = 0;
    float numSides = 7;
    float angleStep = (Math::PI * 2) / numSides;
    std::vector<Vector2> vertices;

    for(size_t i = 0; i < numSides; i++)
    {
        vertices.emplace_back(
            std::cos(angle) * radius,
            std::sin(angle) * radius
        );

        angle += angleStep;
    }

    b = ConvexPolygonCollider(vertices);
    bTransform.position = Vector2(500, 500);
}

void CapsulePolygonCollisionDetectionWindow::Render()
{
    ImGui::Begin("Capsule-Polygon Collision");
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    Vector2 mousePosition(ImGui::GetMousePos().x, ImGui::GetMousePos().y);

    aTransform.position = mousePosition;

    DrawCapsule(drawList, &a, &aTransform, WHITE);
    DrawPolygon(drawList, &b, &bTransform);

    float rotationAmount = 0.05;

    if(ImGui::IsKeyDown(ImGuiKey::ImGuiKey_Q))
        aTransform.rotation -= rotationAmount;
    else if(ImGui::IsKeyDown(ImGuiKey::ImGuiKey_E))
        aTransform.rotation += rotationAmount;

    float aRadius = a.GetWidth() / 2.0f;
    Line centerLine = a.GetCenterLine(&aTransform);
    std::vector<Vector2> centerLineVertices = {centerLine.start, centerLine.end};
    std::vector<Vector2> bPoints = b.transformPoints(&bTransform);
    std::vector<Line> bEdges = b.getEdges(&bTransform);

    ClosestVertexProjection closestProjection = centerLine.closestVertexOnLine(bPoints);
    Line closestEdge = bEdges[0];
    float closestEdgeDistance = Math::FLOAT_MAX;

    for (Line &edge : bEdges)
    {
        ClosestVertexProjection projection = edge.closestVertexOnLine(centerLineVertices);

        if (projection.distance < closestProjection.distance)
            closestProjection = projection;

        Vector2 edgeCenter = edge.start + ((edge.end - edge.start) / 2.0f);
        float edgeDistance = Vector2::distanceSquared(aTransform.position, edgeCenter);

        if (edgeDistance < closestEdgeDistance)
        {
            closestEdge = edge;
            closestEdgeDistance = edgeDistance;
        }
    }

    Vector2 capsuleLineDir = (centerLine.end - centerLine.start).normalize();
    Vector2 edgeDir = (closestEdge.end - closestEdge.start).normalize();

    //Draw closest projections
    drawList->AddLine(toImVec2(centerLine.start), toImVec2(centerLine.end), BLACK, 1.0f);
    drawList->AddLine(toImVec2(closestProjection.vertex), toImVec2(closestProjection.projectedPoint), RED);
    drawList->AddLine(toImVec2(closestEdge.start), toImVec2(closestEdge.end), RED, 2.0f);

    if (closestProjection.distance < aRadius)
    {
        Vector2 normal = (closestProjection.projectedPoint - closestProjection.vertex).normalize();
        float depth = aRadius - closestProjection.distance;

        Vector2 bodyDir = bTransform.position - aTransform.position;
        if (Vector2::dot(bodyDir, normal) < 0)
            normal *= -1;

        Transform resolvedCapsuleTransform = aTransform;
        resolvedCapsuleTransform.position -= normal * depth;
        DrawCapsule(drawList, &a, &resolvedCapsuleTransform, BLUE);
        CollisionPoints collisionPoints;
        CollisionAlgorithms::GenerateCapsulePolygonContactPoints(
            &a, &resolvedCapsuleTransform, &b, &bTransform, &collisionPoints
        );

        for(Vector2& contact : collisionPoints.contacts)
        {
            //Draw contact
            drawList->AddCircleFilled(toImVec2(contact), 5.0f, RED);

            //Draw penetration depth
            Vector2 penetrationDepth = contact + (normal * depth);
            drawList->AddLine(toImVec2(contact), toImVec2(penetrationDepth), RED, 2.0f);
        }
    }


    

    ImGui::End();
}