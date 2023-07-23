#pragma once

#include <windows/Window.h>
#include <Transform.h>
#include <collision/colliders/CapsuleCollider.h>

class CapsuleCapsuleCollisionDetectionWindow : public Window
{
public:
    CapsuleCapsuleCollisionDetectionWindow();
    void Render() override;

private:
    CapsuleCollider a;
    CapsuleCollider b;
    Transform aTransform;
    Transform bTransform;
};