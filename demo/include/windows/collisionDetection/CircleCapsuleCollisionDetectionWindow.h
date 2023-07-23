#pragma once

#include <windows/Window.h>
#include <collision/colliders/CircleCollider.h>
#include <collision/colliders/CapsuleCollider.h>
#include <Transform.h>

class CircleCapsuleCollisionDetectionWindow : public Window
{
public:
    CircleCapsuleCollisionDetectionWindow();
    void Render() override;

private:
    CircleCollider a;
    CapsuleCollider b;
    Transform aTransform;
    Transform bTransform;
};