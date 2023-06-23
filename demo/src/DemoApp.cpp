#include <iostream>
#include "DemoApp.h"
#include <colliders/CircleCollider.h>
#include <colliders/LineCollider.h>
#include <colliders/BoxCollider.h>
#include <core/Time.h>
#include <core/Logger.h>
#include <imgui.h>
#include <Vector2.h>
#include <cstdlib>

DemoApp::DemoApp()
{
}

void DemoApp::run()
{
    // createLine(Vector2(100.0f, 100.0f), 100.0f);
    // createLine(Vector2(100.0f, 125.0f), 100.0f);

    createBox(Vector2(100.0f, 100.0f), 200, 100);
    // createBox(Vector2(250.0f, 250.0f), 200, 100);

    createCircle(40.0f, Vector2(300.0f, 300.0f), 1.0f, 1.0f);
    // createCircle(20.0f, Vector2(200.0f, 100.0f), 0.0f, 1.0f);
    //createCircle(40.0f, Vector2(300.0f, 100.0f), 1.0f, 1.0f, true);
    //createLine(Vector2(300, 400), Vector2(600, 400));

    double dt = 0;
    float movementSpeed = 10.0f;
    float angularSpeed = 0.2f;
    entities[1]->rigidBody->addAngularForce(angularSpeed);

    while (renderer.running())
    {
        double startTime = Time::time();

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

        // if(ImGui::IsMouseDown(0))
        // {
        //     float size = rand() % 50;
        //     createCircle(size, renderer.getMousePosition(), 1.0f, 1.0f);
        // }

        entities[0]->rigidBody->addForce(force);
        entities[0]->rigidBody->addAngularForce(angularForce);

        world.step(dt);
        renderer.render(entities, world.getCollisions());

        dt = Time::time() - startTime;
    }
}

void DemoApp::createCircle(float radius, Vector2 position, float mass, float elasticity, bool isStatic)
{
    std::shared_ptr<CircleCollider> circleCollider = std::make_shared<CircleCollider>(radius);
    std::shared_ptr<RigidBody> rigidBody = std::make_shared<RigidBody>(circleCollider.get());
    rigidBody->setStatic(isStatic);
    rigidBody->setMass(mass);
    rigidBody->setElasticity(elasticity);
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

void DemoApp::createBox(Vector2 position, float width, float height)
{
    std::shared_ptr<BoxCollider> boxCollider = std::make_shared<BoxCollider>(width, height);
    std::shared_ptr<RigidBody> rigidBody = std::make_shared<RigidBody>(boxCollider.get());
    rigidBody->transform.position = position;

    std::shared_ptr<Entity> entity = std::make_shared<Entity>();
    entity->rigidBody = rigidBody;
    entity->collider = boxCollider;

    entities.push_back(entity);
    world.addRigidBody(rigidBody.get());
}