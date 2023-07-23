#pragma once

#include <windows/Window.h>
#include <Transform.h>
#include <collision/colliders/BoxCollider.h>
#include <collision/colliders/CircleCollider.h>

class CirclePolygonCollisionDetectionWindow : public Window
{
public:
    CirclePolygonCollisionDetectionWindow();
    void Render() override;
    ImVec2 toImVec2(Vector2 v) override;

private:
    CircleCollider a;
    BoxCollider b;
    Transform aTransform;
    Transform bTransform;
    Vector2 windowOffset = Vector2(900.0, 500.0);
};