#include <iostream>
#include <vector>
#include <cstdlib>
#include <thread>

#include <imgui.h>

#include "DemoApp.h"
#include <collision/colliders/CircleCollider.h>
#include <collision/colliders/LineCollider.h>
#include <collision/colliders/BoxCollider.h>
#include <collision/colliders/ConvexPolygonCollider.h>
#include <core/Time.h>
#include <core/Logger.h>
#include <Vector2.h>
#include <core/Timer.h>

DemoApp::DemoApp()
{
}

void DemoApp::run()
{
    // createBox(Vector2(0.0f, 0.0f), 100, 100, 0.0f, 100.0f, 0.0f);
    createNGon(Vector2(0, 0), 50, 3, 10.f);
    // createCircle(100.0f, Vector2(100.0f, 100.0f), 1.0f, 0.0f, true);
    // createBox(Vector2(-200, 100.0f), 100, 100, 0.0f, 100.0f, 0.0f, true);

    // createLine(Vector2(100.0f, 100.0f), 100.0f);
    // createLine(Vector2(100.0f, 125.0f), 100.0f);

    // createBox(Vector2(100.0f, 100.0f), 200, 100, 0.0f, 100.0f, 0.0f);

    // Static boxes
    createBox(Vector2(500.0f, 800.0f), 5000, 50, 0.0f, 0.0f, 0.0f, true);
    createBox(Vector2(200.0f, 200.0f), 1500, 20, 0.2f, 0.0f, 0.0f, true);
    createBox(Vector2(1200.0f, 500.0f), 1500, 20, -0.2f, 0.0f, 0.0f, true);

    // createCircle(20.0f, Vector2(200.0f, 300.1f), 0.0f, 0.0f);

    float movementSpeed = 250.0f;
    float angularSpeed = 10.0f;
    
    // entities[1]->rigidBody->addAngularForce(angularSpeed);
    
    double currentTime = Time::time();
    double accumulator = 0.0f; 

    while (renderer.running())
    {
        Timer::start("App Loop");
        double newTime = Time::time();
        double frameTime = newTime - currentTime;
        currentTime = newTime;

        accumulator += frameTime;

        Vector2 force = Vector2(0,0);
        float angularForce = 0.0f;

        if(ImGui::IsKeyDown(ImGuiKey_A))
            force.x = -movementSpeed;
        
        if(ImGui::IsKeyDown(ImGuiKey_D))
            force.x = movementSpeed;
        
        if(ImGui::IsKeyDown(ImGuiKey_W))
            force.y = -movementSpeed;
        
        if(ImGui::IsKeyDown(ImGuiKey_S))
            force.y = movementSpeed;

        if(ImGui::IsKeyDown(ImGuiKey_Q))
            angularForce = -angularSpeed;

        if(ImGui::IsKeyDown(ImGuiKey_E))
            angularForce = angularSpeed;

        if(ImGui::IsMouseClicked(0))
        {
            float radius = rand() % 50;
            createCircle(radius, renderer.getMousePosition(), 10.0f, 0.f);
        }
        else if(ImGui::IsMouseClicked(1))
        {
            float width = (rand() % 150) + 50.0f;
            float height = (rand() % 150) + 50.0f;
            createBox(renderer.getMousePosition(), width, height, 0.0f, 10.0f, 0.0f);
        }
        else if(ImGui::IsMouseClicked(2))
        {
            float radius = rand() % 50;
            int numSides = (rand() % 8) + 3;
            // float radius = 50;
            // int numSides = 5;
            createNGon(renderer.getMousePosition(), radius, numSides, 10.f, 0.0f);
        }

        entities[0]->rigidBody->addForce(force);
        entities[0]->rigidBody->addAngularForce(angularForce);

        // //Interesting way of doing fixed timesteps but becomes slugish after a while
        // while(accumulator > TIME_STEP)
        // {
        //     world.step(TIME_STEP);
        //     accumulator -= TIME_STEP;
        // }

        world.step(frameTime);

        Timer::stop("App Loop");
        renderer.render(entities, world.getCollisions());
    }
}

void DemoApp::createCircle(float radius, Vector2 position, float mass, float restitution, bool isStatic)
{
    std::shared_ptr<CircleCollider> circleCollider = std::make_shared<CircleCollider>(radius);
    std::shared_ptr<RigidBody> rigidBody = std::make_shared<RigidBody>(circleCollider.get());
    rigidBody->setStatic(isStatic);
    rigidBody->setMass(mass);
    rigidBody->setRestitution(restitution);
    rigidBody->transform.position.set(position); 

    std::shared_ptr<Entity> entity = std::make_shared<Entity>();
    entity->rigidBody = rigidBody;
    entity->collider = circleCollider;

    entities.push_back(entity);
    world.addRigidBody(rigidBody.get());
}

void DemoApp::createLine(Vector2 position, float width, bool isStatic)
{
    std::shared_ptr<LineCollider> lineCollider = std::make_shared<LineCollider>(position, width);
    std::shared_ptr<RigidBody> rigidBody = std::make_shared<RigidBody>(lineCollider.get());
    rigidBody->setStatic(isStatic);
    rigidBody->transform.position = position;

    std::shared_ptr<Entity> entity = std::make_shared<Entity>();
    entity->rigidBody = rigidBody;
    entity->collider = lineCollider;
    
    entities.push_back(entity);
    world.addRigidBody(rigidBody.get());
}

void DemoApp::createBox(Vector2 position, float width, float height, float rotation, float mass, float restitution, bool isStatic)
{
    std::shared_ptr<BoxCollider> boxCollider = std::make_shared<BoxCollider>(width, height);
    std::shared_ptr<RigidBody> rigidBody = std::make_shared<RigidBody>(boxCollider.get());
    rigidBody->transform.position = position;
    rigidBody->transform.rotation = rotation;
    rigidBody->setMass(mass);
    rigidBody->setRestitution(restitution);
    rigidBody->setStatic(isStatic);

    std::shared_ptr<Entity> entity = std::make_shared<Entity>();
    entity->rigidBody = rigidBody;
    entity->collider = boxCollider;

    entities.push_back(entity);
    world.addRigidBody(rigidBody.get());
}

void DemoApp::createNGon(Vector2 position, float radius, size_t numSides, float mass, float restitution, bool isStatic)
{
    float angle = 0;
    float angleStep = (Math::PI * 2) / numSides;
    std::vector<Vector2> vertices;

    for(size_t i = 0; i < numSides; i++)
    {
        float offset = rand() % 50;

        vertices.emplace_back(
            std::cos(angle) * (radius + offset),
            std::sin(angle) * (radius + offset)
        );

        angle += angleStep;
    }

    std::shared_ptr<ConvexPolygonCollider> polygonCollider = std::make_shared<ConvexPolygonCollider>(vertices);
    std::shared_ptr<RigidBody> rigidBody = std::make_shared<RigidBody>(polygonCollider.get());
    rigidBody->transform.position = position;
    rigidBody->setMass(mass);
    rigidBody->setRestitution(restitution);
    rigidBody->setStatic(isStatic);

    std::shared_ptr<Entity> entity = std::make_shared<Entity>();
    entity->rigidBody = rigidBody;
    entity->collider = polygonCollider;

    entities.push_back(entity);
    world.addRigidBody(rigidBody.get());
}