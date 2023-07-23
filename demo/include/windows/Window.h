#pragma once

#include <imgui.h>
#include <Constants.h>
#include <math/Vector2.h>
#include <Transform.h>
#include <collision/colliders/CapsuleCollider.h>
#include <collision/colliders/ConvexPolygonCollider.h>
#include <collision/colliders/BoxCollider.h>

class Window
{
public:
    virtual void Render() = 0;

protected:
    virtual ImVec2 toImVec2(Vector2 v);
    void DrawCircleOnAxis(ImDrawList* drawList, Vector2 axis, float radius, Vector2 position);
    void DrawPolygonOnAxis(ImDrawList* drawList, Vector2 axis, std::vector<Vector2> vertices);
    void DrawAxis(ImDrawList* drawList, Vector2 axis, float length=1000.0f);
    void DrawCapsule(ImDrawList* drawList, CapsuleCollider* capsule, Transform* transform, ImU32 colour);
    void DrawBox(ImDrawList* drawList, BoxCollider* box, Transform* transform, ImU32 colour=WHITE);
    void DrawAABB(ImDrawList* drawList, AABB *aabb, ImU32 colour);
    void DrawPolygon(ImDrawList* drawList, ConvexPolygonCollider* polygon, Transform* transform, ImU32 colour=WHITE);
};