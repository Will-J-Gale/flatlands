#pragma once

#include <vector>
#include <memory>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>

#include <collision/colliders/CircleCollider.h>
#include <collision/colliders/LineCollider.h>
#include <collision/colliders/BoxCollider.h>
#include <collision/colliders/ConvexPolygonCollider.h>
#include <collision/colliders/CapsuleCollider.h>
#include <Vector2.h>
#include <Entity.h>
#include <collision/Collision.h>

class Renderer
{
public:
    Renderer();
    ~Renderer();
    
    void render(std::vector<std::shared_ptr<Entity>>& entities, std::vector<Collision>* collisions);
    void destroy();
    bool running();
    void renderRigidBody(ImDrawList* drawList, const Entity* body);
    Vector2 getMousePosition();
    ImVec2 toImVec2(Vector2 v);

private:
    void drawCircleOnAxis(ImDrawList* drawList, Vector2 axis, float radius, Vector2 position);
    void drawPolygonOnAxis(ImDrawList* drawList, Vector2 axis, std::vector<Vector2> vertices);
    void drawCollisionDetection(ImDrawList* drawList, std::vector<std::shared_ptr<Entity>>& entities);
    void drawAxis(ImDrawList* drawList, Vector2 axis, float length=1000.0f);
private:
    GLFWwindow* window;
    float dt = 0;
    Vector2 mousePosition = Vector2(0,0);
    ImVec2 windowSize = ImVec2(0, 0);
    bool renderDebug = false;

    void drawCapsule(ImDrawList* drawList, CapsuleCollider* capsule, Transform* transform, ImU32 colour);
    void drawBox(ImDrawList* drawList, BoxCollider* box, Transform* transform);
};