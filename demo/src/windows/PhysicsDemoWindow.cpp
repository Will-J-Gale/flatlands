#include <windows/PhysicsDemoWindow.h>
#include <core/Time.h>
#include <collision/colliders/CircleCollider.h>
#include <collision/colliders/LineCollider.h>
#include <collision/colliders/BoxCollider.h>
#include <collision/colliders/ConvexPolygonCollider.h>
#include <collision/colliders/CapsuleCollider.h>
#include <collision/broadPhase/QuadTree.h>
#include <math/Vector2.h>
#include <Entity.h>
#include <collision/Collision.h>
#include <Constants.h>
#include <core/Metrics.h>
#include <collision/broadPhase/QuadTreeDetection.h>
#include <collision/broadPhase/NaiveAABBDetection.h>

PhysicsDemoWindow::PhysicsDemoWindow()
{
    auto broadPhase = std::make_shared<NaiveAABBDetection>();
    // auto broadPhase = std::make_shared<QuadTreeDetection>(AABB(Vector2(0, 0), Vector2(1920, 1080)), 4);
    // auto broadPhase = std::make_shared<SpatialHashDetection>(100, 2000);
    world.SetBroadPhase(broadPhase);

    // Static elements
    CreateBox(Vector2(800.0f, 850.0f), 5000, 50, 0.0f, 0.0f, 0.0f, true);
    CreateLine(Vector2(0, 200), Vector2(950, 400), true);
    CreateLine(Vector2(1920, 400), Vector2(500, 700), true);
    CreateLine(Vector2(0, 0), Vector2(0, 1080), true);
    CreateLine(Vector2(1920, 0), Vector2(1920, 1080), true);

    currentTime = Time::time();
}

void PhysicsDemoWindow::Render()
{
    ImGui::Begin("Physics Demo");

    ImDrawList* drawList = ImGui::GetWindowDrawList();
    double newTime = Time::time();
    double frameTime = newTime - currentTime;
    currentTime = newTime;
    ImVec2 imGuiMousePos = ImGui::GetMousePos();
    Vector2 mousePosition = Vector2(imGuiMousePos.x, imGuiMousePos.y);

    if(ImGui::IsMouseClicked(0))
    {
        float radius = rand() % 50; 
        CreateCircle(radius, mousePosition, 10.0f, 0.f);
    }

    if(ImGui::IsMouseClicked(1))
    {
        float r = rand() % 2;

        if(r == 0)
        {
            float radius = (rand() % 45) + 10;
            int numSides = (rand() % 5) + 20;
            CreateNGon(mousePosition, radius, numSides, 10.0f);
        }
        else
        {
            float width = (rand() % 90) + 10;
            float height = (rand() % 90) + 10;
            CreateBox(mousePosition, width, height, 0.0f, 10.0f);
        }
    }
    else if(ImGui::IsMouseClicked(2))
    {
        float width = (rand() % 90) + 10;
        float height = width * 2.0f;
        CreateCapsule(mousePosition, width, height, 0.0f, 10.0f);
    }

    world.Step(frameTime);

    for(auto entity : entities)
    {
        RenderRigidBody(drawList, entity.get());
    }

    if(renderDebug)
    {
        std::vector<Collision>* collisions = world.GetCollisions();
        for (Collision collision : *collisions)
        {
            for(Vector2 contact : collision.collisionPoints.contacts)
            {
                drawList->AddCircleFilled(toImVec2(contact), 5.0f, RED);

                Vector2 normalEnd = contact + (collision.collisionPoints.normal * 25.0f);
                drawList->AddLine(toImVec2(contact), toImVec2(normalEnd), RED);
            }
        }
    }

    auto worldMetrics = world.GetMetrics();
    ImGui::Checkbox("Render Debug", &renderDebug);
    ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::Text(std::string("Entities: " + std::to_string(worldMetrics.numEntities)).c_str());
    ImGui::Text(std::string("Broad phase checks: " + std::to_string(worldMetrics.broadPhaseChecks)).c_str());
    ImGui::Text(std::string("Narrow phase checks: " + std::to_string(worldMetrics.narrowPhaseChecks)).c_str());
    ImGui::Spacing();
    ImGui::Text(std::string("Step time: " + std::to_string(worldMetrics.worldStepTime)).c_str());
    ImGui::Text(std::string("Move bodies: " + std::to_string(worldMetrics.moveBodiesTime)).c_str());
    ImGui::Text(std::string("Broad phase time: " + std::to_string(worldMetrics.broadPhaseTime)).c_str());
    ImGui::Text(std::string("Narrow phase time: " + std::to_string(worldMetrics.narrowPhaseTime)).c_str());
    ImGui::Text(std::string("Resolve collisions: " + std::to_string(worldMetrics.resolveCollisionsTime)).c_str());
    ImGui::End();
}

void PhysicsDemoWindow::RenderRigidBody(ImDrawList *drawList, const Entity *entity)
{
    RigidBody *rigidBody = entity->rigidBody.get();
    Collider *collider = entity->rigidBody->collider;
    Transform *transform = &entity->rigidBody->transform;
    Vector2 pos = transform->position;
    float rotation = transform->rotation;
    ImU32 bodyColour = rigidBody->isAwake ? WHITE : WHITE_A;

    if (renderDebug)
    {
        AABB aabb = entity->collider->GetAABB(&entity->rigidBody->transform);
        DrawAABB(drawList, &aabb, GREEN);
    }

    if (dynamic_cast<CircleCollider *>(collider))
    {
        CircleCollider *circleCollider = (CircleCollider *)collider;
        float radius = circleCollider->radius;
        Vector2 dir = Vector2(radius * std::cos(rotation), radius * std::sin(rotation));

        drawList->AddCircleFilled(toImVec2(pos), radius, bodyColour);
        drawList->AddLine(toImVec2(pos), toImVec2(pos + dir), BLACK);
    }

    else if (dynamic_cast<LineCollider *>(collider))
    {
        LineCollider *lineCollider = (LineCollider *)collider;
        std::vector<Vector2> points = lineCollider->transformPoints(transform);

        drawList->AddLine(toImVec2(points[0]), toImVec2(points[1]), bodyColour);
    }
    else if (dynamic_cast<BoxCollider *>(collider))
    {
        BoxCollider *boxCollider = (BoxCollider *)collider;
        DrawBox(drawList, boxCollider, transform, bodyColour);
    }
    else if (dynamic_cast<LineCollider *>(collider))
    {
        LineCollider *lineCollider = dynamic_cast<LineCollider *>(collider);
        Line line = lineCollider->transformLine(transform);
        drawList->AddLine(toImVec2(line.start), toImVec2(line.end), bodyColour, 2.0f);
    }
    else if (dynamic_cast<ConvexPolygonCollider *>(collider))
    {
        ConvexPolygonCollider *polyCollider = dynamic_cast<ConvexPolygonCollider *>(collider);

        std::vector<Vector2> vertices = polyCollider->transformPoints(transform);
        std::vector<ImVec2> pointsToDraw;

        for (size_t i = 0; i < vertices.size(); i++)
        {
            pointsToDraw.push_back(std::move(toImVec2(vertices[i])));
        };

        drawList->AddConvexPolyFilled(&pointsToDraw[0], vertices.size(), bodyColour);
    }
    else if (dynamic_cast<CapsuleCollider *>(collider))
    {
        CapsuleCollider *capsuleCollider = dynamic_cast<CapsuleCollider *>(collider);

        DrawCapsule(drawList, capsuleCollider, transform, bodyColour);
    }
}

void PhysicsDemoWindow::CreateCircle(float radius, Vector2 position, float mass, float restitution, bool isStatic)
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

void PhysicsDemoWindow::CreateLine(Vector2 start, Vector2 end, bool isStatic)
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

void PhysicsDemoWindow::CreateBox(Vector2 position, float width, float height, float rotation, float mass, float restitution, bool isStatic)
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

void PhysicsDemoWindow::CreateNGon(Vector2 position, float radius, size_t numSides, float mass, float restitution, bool isStatic)
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

void PhysicsDemoWindow::CreateCapsule(Vector2 position, float width, float height, float rotation, float mass, float restitution, bool isStatic)
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

