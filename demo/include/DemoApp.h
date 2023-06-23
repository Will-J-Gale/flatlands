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
    void createLine(Vector2 position, float width, bool isStatic=true);
    void createBox(Vector2 position, float width, float height);

    Renderer renderer;
    World world = World(GRAVITY);
    std::vector<std::shared_ptr<Entity>> entities;
};
