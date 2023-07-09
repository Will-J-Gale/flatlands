#pragma once

#include <memory>
#include <World.h>
#include <Constants.h>
#include <RigidBody.h>
#include <Renderer.h>
#include <Vector2.h>
#include <Entity.h>

class DemoApp
{
public:
    DemoApp();
    void run();

private:
    void createCircle(float radius, Vector2 position, float mass=1.0f, float elasticity=1.0f, bool isStatic=false);
    void createLine(Vector2 start, Vector2 end, bool isStatic=false);
    void createBox(Vector2 position, float width, float height, float rotation=0.0f, float mass=1.0, float restitution=1.0, bool isStatic=false);
    void createNGon(Vector2 position, float radius, size_t numSides, float mass=1.0f, float restitution=0.2f, bool isStatic=false);
    void createCapsule(Vector2 position, float width, float height, float rotation=0.0f, float mass=1.0f, float restitution=0.2f, bool isStatic=false);

    Renderer renderer;
    World world = World(GRAVITY, NUM_ITRERATIONS);
    std::vector<std::shared_ptr<Entity>> entities;
};
