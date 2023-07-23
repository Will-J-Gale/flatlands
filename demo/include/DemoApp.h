#pragma once

#include <memory>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>

#include <World.h>
#include <Constants.h>
#include <RigidBody.h>
#include <math/Vector2.h>
#include <Entity.h>
#include <windows/Window.h>
#include <windows/PhysicsDemoWindow.h>
#include <windows/DemoSelectWindow.h>

class DemoApp
{
public:
    DemoApp();
    void Run();
    void Destroy();

private:
    void OnDemoSelected(Demo demo);

private:
    GLFWwindow* window;
    DemoSelectWindow demoSelectWindow;
    std::unique_ptr<Window> demoWindow = std::make_unique<PhysicsDemoWindow>();
};
