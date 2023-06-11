#pragma once

#include <vector>
#include <RigidBody.h>
#include <Vector2.h>
#include <colliders/Collision.h>

class World
{
public:
    World(Vector2 gravity);
    void step(float dt);
    void addRigidBody(RigidBody* rigidBody);
    std::vector<RigidBody*> getBodies();
    std::vector<Collision>* getCollisions();
    
private:
    void resolveCollisions();

    std::vector<RigidBody*> bodies;
    std::vector<Collision> collisions;
    Vector2 gravity;
    float dt = 0;
};