#pragma once

#include <vector>
#include <memory>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#include <Vector2.h>
#include <Entity.h>
#include <collision/Collision.h>

class Renderer
{
public:
    Renderer();
    
    void render(std::vector<std::shared_ptr<Entity>>& entities, std::vector<Collision>* collisions);
    void destroy();
    bool running();
    void renderRigidBody(ImDrawList* drawList, const Entity* body);
    Vector2 getMousePosition();
    ImVec2 toImVec2(Vector2 v);

private:
    GLFWwindow* window;
    float dt = 0;
    Vector2 mousePosition = Vector2(0,0);
    ImVec2 windowSize = ImVec2(0, 0);
    bool renderDebug = false;
};