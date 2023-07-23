#pragma once

#include <vector>
#include <Constants.h>
#include <World.h>
#include <math/Vector2.h>
#include <windows/Window.h>
#include <Entity.h>

class PhysicsDemoWindow : public Window
{
public:
    PhysicsDemoWindow();
    void Render() override;

private:
    void RenderRigidBody(ImDrawList* drawList, const Entity* body);
    void CreateCircle(float radius, Vector2 position, float mass=1.0f, float elasticity=1.0f, bool isStatic=false);
    void CreateLine(Vector2 start, Vector2 end, bool isStatic=false);
    void CreateBox(Vector2 position, float width, float height, float rotation=0.0f, float mass=1.0, float restitution=1.0, bool isStatic=false);
    void CreateNGon(Vector2 position, float radius, size_t numSides, float mass=1.0f, float restitution=0.2f, bool isStatic=false);
    void CreateCapsule(Vector2 position, float width, float height, float rotation=0.0f, float mass=1.0f, float restitution=0.2f, bool isStatic=false);

    // void DrawCapsuleCapsuleCollisionTest(ImDrawList* drawList, CapsuleCollider* a, Transform* aTransform, CapsuleCollider* b, Transform* bTransform);
    // void DrawCircleCapsuleCollisionTest(ImDrawList* drawList, CircleCollider* a, Transform* aTransform, CapsuleCollider* b, Transform* bTransform);
    // void DrawCapsulePolygonCollision(ImDrawList* drawList, CapsuleCollider* a, Transform* aTransform, ConvexPolygonCollider* b, Transform* bTransform);

private:
    World world = World(GRAVITY, NUM_ITRERATIONS);
    std::vector<std::shared_ptr<Entity>> entities;
    double currentTime = 0.0;
    bool renderDebug = false;
};

