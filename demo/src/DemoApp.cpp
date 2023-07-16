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
#include <collision/colliders/CapsuleCollider.h>
#include <core/Time.h>
#include <core/Logger.h>
#include <math/Vector2.h>
#include <core/Timer.h>
#include <collision/broadPhase/NaiveAABBDetection.h>
#include <collision/broadPhase/QuadTreeDetection.h>
#include <collision/broadPhase/SpatialHashDetection.h>

DemoApp::DemoApp()
{
}

void DemoApp::run()
{
    // auto broadPhase = std::make_shared<NaiveAABBDetection>();
    // auto broadPhase = std::make_shared<QuadTreeDetection>(AABB(Vector2(0, 0), Vector2(1920, 1080)), 4);
    auto broadPhase = std::make_shared<SpatialHashDetection>(100, 2000);
    world.SetBroadPhase(broadPhase);

    double currentTime = Time::time();

    float radius = rand() % 50; 
    // createCircle(10, Vector2(960, 540), 10.0f, 0.f);
    // createCircle(10, Vector2(920, 540), 10.0f, 0.f);

    // // Static elements
    // createBox(Vector2(500.0f, 200.0f), 100, 50, 0.0f, 0.0f, 0.0f, true);
    // createBox(Vector2(800.0f, 800.0f), 1000, 50, 0.0f, 0.0f, 0.0f, true);
    // createLine(Vector2(0, 150), Vector2(950, 350), true);
    // createLine(Vector2(1920, 350), Vector2(500, 650), true);
    // createLine(Vector2(0, 0), Vector2(0, 1080), true);
    // createLine(Vector2(1920, 0), Vector2(1920, 1080), true);

    while (renderer.running())
    {
        Timer::start("App Loop");
        double newTime = Time::time();
        double frameTime = newTime - currentTime;
        currentTime = newTime;

        float movementSpeed = 20000.0f;

        float angularSpeed = 10000000.0f;
        // float movementSpeed = 3.0f;
        // float angularSpeed = 0.1f;

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

        if(entities.size() > 0)
        {
            entities[0]->rigidBody->AddForce(force);
            entities[0]->rigidBody->AddAngularForce(angularForce);
            // entities[0]->rigidBody->transform.position += force;
            // entities[0]->rigidBody->transform.rotation += angularForce;
        }

        // if(ImGui::IsMouseClicked(0))
        if(ImGui::IsMouseDown(0))
        {
            // float radius = rand() % 50; 
            createCircle(10, renderer.getMousePosition(), 10.0f, 0.f);
        }

        if(ImGui::IsMouseClicked(1))
        {
            float r = rand() % 2;

            if(r == 0)
            {
                float radius = (rand() % 45) + 10;
                int numSides = (rand() % 5) + 3;
                createNGon(renderer.getMousePosition(), radius, numSides, 10.0f);
            }
            else
            {
                float width = (rand() % 90) + 10;
                float height = (rand() % 90) + 10;
                createBox(renderer.getMousePosition(), width, height, 0.0f, 10.0f);
            }
        }
        else if(ImGui::IsMouseClicked(2))
        {
            float width = (rand() % 90) + 10;
            float height = (rand() % 90) + 10;
            createCapsule(renderer.getMousePosition(), width, height, 0.0f, 10.0f);
        }

        world.Step(frameTime);
        world.GetBodies();

        Timer::stop("App Loop");
        renderer.render(entities, world.GetCollisions(), world.GetMetrics(), nullptr);
    }
}

void DemoApp::createCircle(float radius, Vector2 position, float mass, float restitution, bool isStatic)
{
    std::shared_ptr<CircleCollider> circleCollider = std::make_shared<CircleCollider>(radius);
    std::shared_ptr<RigidBody> rigidBody = std::make_shared<RigidBody>(circleCollider.get());
    rigidBody->SetStatic(isStatic);
    rigidBody->SetMass(mass);
    rigidBody->SetRestitution(restitution);
    rigidBody->transform.position.set(position); 

    std::shared_ptr<Entity> entity = std::make_shared<Entity>();
    entity->rigidBody = rigidBody;
    entity->collider = circleCollider;

    entities.push_back(entity);
    world.AddRigidBody(rigidBody.get());
}

void DemoApp::createLine(Vector2 start, Vector2 end, bool isStatic)
{
    std::shared_ptr<LineCollider> lineCollider = std::make_shared<LineCollider>(start, end);
    std::shared_ptr<RigidBody> rigidBody = std::make_shared<RigidBody>(lineCollider.get());
    rigidBody->SetStatic(isStatic);

    Vector2 position = start + ((end - start) / 2.0f);
    rigidBody->transform.position = position;

    std::shared_ptr<Entity> entity = std::make_shared<Entity>();
    entity->rigidBody = rigidBody;
    entity->collider = lineCollider;
    
    entities.push_back(entity);
    world.AddRigidBody(rigidBody.get());
}

void DemoApp::createBox(Vector2 position, float width, float height, float rotation, float mass, float restitution, bool isStatic)
{
    std::shared_ptr<BoxCollider> boxCollider = std::make_shared<BoxCollider>(width, height);
    std::shared_ptr<RigidBody> rigidBody = std::make_shared<RigidBody>(boxCollider.get());
    rigidBody->transform.position = position;
    rigidBody->transform.rotation = rotation;
    rigidBody->SetMass(mass);
    rigidBody->SetRestitution(restitution);
    rigidBody->SetStatic(isStatic);

    std::shared_ptr<Entity> entity = std::make_shared<Entity>();
    entity->rigidBody = rigidBody;
    entity->collider = boxCollider;

    entities.push_back(entity);
    world.AddRigidBody(rigidBody.get());
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
    rigidBody->SetMass(mass);
    rigidBody->SetRestitution(restitution);
    rigidBody->SetStatic(isStatic);

    std::shared_ptr<Entity> entity = std::make_shared<Entity>();
    entity->rigidBody = rigidBody;
    entity->collider = polygonCollider;

    entities.push_back(entity);
    world.AddRigidBody(rigidBody.get());
}

void DemoApp::createCapsule(Vector2 position, float width, float height, float rotation, float mass, float restitution, bool isStatic)
{
    std::shared_ptr<CapsuleCollider> polygonCollider = std::make_shared<CapsuleCollider>(width, height);
    std::shared_ptr<RigidBody> rigidBody = std::make_shared<RigidBody>(polygonCollider.get());
    rigidBody->transform.position = position;
    rigidBody->transform.rotation = rotation;
    rigidBody->SetMass(mass);
    rigidBody->SetRestitution(restitution);
    rigidBody->SetStatic(isStatic);

    std::shared_ptr<Entity> entity = std::make_shared<Entity>();
    entity->rigidBody = rigidBody;
    entity->collider = polygonCollider;

    entities.push_back(entity);
    world.AddRigidBody(rigidBody.get());
}