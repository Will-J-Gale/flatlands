#include <imgui.h>
#include <windows/DemoSelectWindow.h>

DemoSelectWindow::DemoSelectWindow()
{
}

void DemoSelectWindow::Render()
{
    ImGui::Begin("Demo select");
    {
        static ImGuiComboFlags flags = 0;
        if (ImGui::BeginCombo("Demo select", "Select a demo", flags))
        {
            bool is_selected = false;
            Demo demo = Demo::None;

            if (ImGui::Selectable("Physics Demo"))
                demo = Demo::PhysicsDemo;
            else if(ImGui::Selectable("Circle-Circle Collision"))
                demo = Demo::CircleCircle;
            else if(ImGui::Selectable("Circle-Polygon Collision"))
                demo = Demo::CirclePolygon;
            else if(ImGui::Selectable("Capsule-Circle Collision"))
                demo = Demo::CapsuleCircle;
            else if(ImGui::Selectable("Capsule-Polygon Collision"))
                demo = Demo::CapsulePolygon;
            else if(ImGui::Selectable("Capsule-Capsule Collision"))
                demo = Demo::CapsuleCapsule;
            
            if(demo != Demo::None and demoSelectedCallback != nullptr)
                demoSelectedCallback(demo);

            ImGui::EndCombo();
        }
    }
    ImGui::End();
}

void DemoSelectWindow::SetDemoSelectCallback(OnDemoSelectedCallback demoSelectedCallback)
{
    this->demoSelectedCallback = demoSelectedCallback;
}