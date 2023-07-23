#include <windows/collisionDetection/CircleCircleCollisionDetectionWindow.h>

void CircleCircleCollisionDetectionWindow::Render()
{
    ImGui::Begin("Circle-Circle Collision");

    Vector2 staticCirclePos(500, 500);
    float staticCircleRadius = 100.0f;

    ImU32 mouseColour = GREEN;
    ImVec2 imGuiMousePos = ImGui::GetMousePos();
    Vector2 mousePosition(imGuiMousePos.x, imGuiMousePos.y);
    float mouseRadius = 50.0f;

    ImDrawList* drawList = ImGui::GetWindowDrawList();
    
    float radiusSum = staticCircleRadius + mouseRadius;
    float distance = Vector2::distance(staticCirclePos, mousePosition);
    bool collision = false;

    if(distance < radiusSum)
    {
        collision = true;
        mouseColour = GREEN_A;
    }

    drawList->AddCircleFilled(toImVec2(staticCirclePos), staticCircleRadius, WHITE);
    drawList->AddCircleFilled(toImVec2(mousePosition), mouseRadius, mouseColour);

    if(collision)
    {
        Vector2 normal = (staticCirclePos - mousePosition).normalize();
        float penetrationDepth = radiusSum - distance;

        //Draw circle penetration resolution and contact point
        Vector2 resolvedCirclePos = mousePosition - (normal * penetrationDepth);
        Vector2 contact = resolvedCirclePos + (normal * mouseRadius);
        drawList->AddCircleFilled(toImVec2(contact), 5.0f, RED);
        drawList->AddCircleFilled(toImVec2(resolvedCirclePos), mouseRadius, CYAN);
        
        //Draw normal + penetration depth
        Vector2 normalEnd = contact + (normal) * penetrationDepth;
        drawList->AddLine(toImVec2(contact), toImVec2(normalEnd), RED, 2.0f);
    }

    ImGui::End();
}