#pragma once

#include <windows/Window.h>
#include <collision/colliders/CapsuleCollider.h>
#include <collision/colliders/ConvexPolygonCollider.h>
#include <Transform.h>

class CapsulePolygonCollisionDetectionWindow : public Window
{
public:
    CapsulePolygonCollisionDetectionWindow();
    void Render() override;

private:
    CapsuleCollider a;
    ConvexPolygonCollider b;
    Transform aTransform;
    Transform bTransform;
};