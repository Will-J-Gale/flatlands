#include <stdio.h>
#include <Renderer.h>
#include <collision/colliders/CircleCollider.h>
#include <collision/colliders/LineCollider.h>
#include <collision/colliders/BoxCollider.h>
#include <collision/colliders/ConvexPolygonCollider.h>
#include <core/Logger.h>
#include <collision/CollisionAlgorithms.h>
#include <Constants.h>
#include <core/Timer.h>

inline static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

void Renderer::drawCircleOnAxis(ImDrawList* drawList, Vector2 axis, float radius, Vector2 position)
{
    auto circleProjection = CollisionAlgorithms::projectCircleOnToAxis(radius, position, axis);
    drawList->AddCircleFilled(toImVec2(axis * circleProjection.min), 4.0f, RED);
    drawList->AddCircleFilled(toImVec2(axis * circleProjection.max), 7.0f, RED);
}

void Renderer::drawPolygonOnAxis(ImDrawList* drawList, Vector2 axis, std::vector<Vector2> vertices)
{
    auto polygonProjection = CollisionAlgorithms::projectShapeOntoAxis(axis, vertices);
    drawList->AddCircleFilled(toImVec2(axis * polygonProjection.min), 4.0f, GREEN);
    drawList->AddCircleFilled(toImVec2(axis * polygonProjection.max), 7.0f, GREEN);   
}
void Renderer::drawAxis(ImDrawList* drawList, Vector2 axis, float length)
{
    Vector2 positiveEnd = axis * length; 
    Vector2 negativeEnd = -axis * length; 
    drawList->AddLine(toImVec2(Vector2()), toImVec2(positiveEnd), BLUE, 2.0f);
    drawList->AddLine(toImVec2(Vector2()), toImVec2(negativeEnd), WHITE, 2.0f);
}

void Renderer::drawCollisionDetection(ImDrawList* drawList, std::vector<std::shared_ptr<Entity>>& entities)
{
    //Draw circle/polygon axis projections
    Entity* entity =  entities[0].get();
    Entity* circle = entities[1].get();

    ConvexPolygonCollider* polyCollider = dynamic_cast<ConvexPolygonCollider*>(entity->collider.get());
    Transform* polyTransform = &entity->rigidBody->transform;

    CircleCollider* circleCollider = dynamic_cast<CircleCollider*>(circle->collider.get());
    Transform* circleTransform = &circle->rigidBody->transform;

    // for(Vector2& axis : polyCollider->getAxes(transform.rotation))
    // {
    //     float axisLength = 1000.0f;
    //     Vector2 positiveEnd = axis * axisLength; 
    //     Vector2 negativeEnd = -axis * axisLength; 
    //     drawList->AddLine(toImVec2(Vector2()), toImVec2(positiveEnd), BLUE, 2.0f);
    //     drawList->AddLine(toImVec2(Vector2()), toImVec2(negativeEnd), WHITE, 2.0f);
        
    //     auto polygonProjection = CollisionAlgorithms::projectShapeOntoAxis(axis, polyCollider->transformPoints(&transform));
    //     drawList->AddCircleFilled(toImVec2(axis * polygonProjection.min), 4.0f, GREEN);
    //     drawList->AddCircleFilled(toImVec2(axis * polygonProjection.max), 7.0f, GREEN);

    //     auto circleProjection = CollisionAlgorithms::projectCircleOnToAxis(circleCollider->radius, circleTransform.position, axis);
    //     drawList->AddCircleFilled(toImVec2(axis * circleProjection.min), 4.0f, RED);
    //     drawList->AddCircleFilled(toImVec2(axis * circleProjection.max), 7.0f, RED);
    // }

    //SAT 2.0
    //Algorithm is a little wonky because Y is flipped from most game engines (Normal up is positive, this down is positive)
    CollisionPoints collisionPoints;
    collisionPoints.depth = Math::FLOAT_MAX;

    //Get shape vertices
    std::vector<Vector2> aPoints = polyCollider->transformPoints(polyTransform);
    std::vector<Vector2> aAxes = polyCollider->getAxes(polyTransform->rotation);

    for(Vector2& axis : aAxes)
    {
        auto aProjection = CollisionAlgorithms::projectShapeOntoAxis(axis, aPoints);
        auto bProjection = CollisionAlgorithms::projectCircleOnToAxis(circleCollider->radius, circleTransform->position, axis);

        Vector2 box_projection = polyTransform->position + (axis * aProjection.min);

        if(aProjection.min >= bProjection.max || bProjection.min >= aProjection.max)
        {
            collisionPoints.hasCollisions = false;
            // return;
        }
        
        // drawAxis(drawList, axis);
        // drawPolygonOnAxis(drawList, axis, aPoints);
        // drawCircleOnAxis(drawList, axis, circleCollider->radius, circleTransform->position);

        float axisDepth = std::min(bProjection.max - aProjection.min, aProjection.max - bProjection.min);

        if(axisDepth < collisionPoints.depth)
        {
            collisionPoints.depth = axisDepth;
            collisionPoints.normal = axis;
        }
    }

    Vector2 closestVertex = CollisionAlgorithms::getClosestVertexToPoint(circleTransform->position, aPoints);
    Vector2 axis = (closestVertex - circleTransform->position).normalize();

    drawList->AddCircleFilled(toImVec2(closestVertex), 10.0f, CYAN);
    auto aProjection = CollisionAlgorithms::projectShapeOntoAxis(axis, aPoints);
    auto bProjection = CollisionAlgorithms::projectCircleOnToAxis(circleCollider->radius, circleTransform->position, axis);

    drawAxis(drawList, axis);
    drawPolygonOnAxis(drawList, axis, aPoints);
    drawCircleOnAxis(drawList, axis, circleCollider->radius, circleTransform->position);
    drawList->AddLine(toImVec2(closestVertex), toImVec2(circleTransform->position), GREEN);

    if(aProjection.min >= bProjection.max || bProjection.min >= aProjection.max)
    {
        collisionPoints.hasCollisions = false;
        return;
    }

    float axisDepth = std::min(bProjection.max - aProjection.min, aProjection.max - bProjection.min);

    if(axisDepth < collisionPoints.depth)
    {
        collisionPoints.depth = axisDepth;
        collisionPoints.normal = axis;
    }

    Vector2 centerA = CollisionAlgorithms::getCenterPoint(aPoints);
    Vector2 dir = circleTransform->position - centerA;

    if(Vector2::dot(dir, collisionPoints.normal) < 0)
        collisionPoints.normal *= -1;

}

Renderer::Renderer()
{
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return;

    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    window = glfwCreateWindow(1920, 1080, "Flatlands", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
}

void Renderer::render(std::vector<std::shared_ptr<Entity>>& entities, std::vector<Collision>* collisions)
{
    Timer::start("Render");
    bool show_demo_window = true;
    ImVec4 clear_color = ImVec4(0.0f, 0.0f, 0.0f, 1.00f);

    // Poll and handle events (inputs, window resize, etc.)
    // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
    // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
    // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
    // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
    glfwPollEvents();

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());
    
    // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
    if (show_demo_window)
        ImGui::ShowDemoWindow(&show_demo_window);

    {
        ImGui::Begin("World");
        windowSize = ImGui::GetWindowSize();

        ImDrawList* drawList = ImGui::GetWindowDrawList();

        ImVec2 windowPos = ImGui::GetWindowPos();
        ImVec2 mousePos = ImGui::GetMousePos();

        mousePosition.x = mousePos.x + windowPos.x;
        mousePosition.y = mousePos.y + windowPos.y;
        
        for(std::shared_ptr<Entity>& entity : entities)
        {
            renderRigidBody(drawList, entity.get());
        }


        if(renderDebug)
        {
            for(Collision& collision : *collisions)
            {
                if(collision.collisionPoints.contacts.size() > 0)
                {
                    for(int i = 0; i < collision.collisionPoints.contacts.size(); i++)
                    {
                        Vector2 contact = collision.collisionPoints.contacts[i];
                        drawList->AddCircleFilled(toImVec2(contact), 10.0f, GREEN);

                        Vector2 frictionImpulse = collision.collisionPoints.frictionImpulses[i];
                        Vector2 impulseEnd = contact + (frictionImpulse * 1);
                        drawList->AddLine(toImVec2(contact), toImVec2(impulseEnd), RED, 5.0f);
                    }
                }
            }
        }

        // drawCollisionDetection(drawList, entities);
        ImGui::End();
    }

    Timer::stop("Render");
    {
        ImGui::Begin("Debug");                          // Create a window called "Hello, world!" and append into it.

        ImGui::Checkbox("Render debug", &renderDebug);
        ImGui::Spacing();

        ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::Text("Entities: %li", entities.size());
        ImGui::Spacing();
        
        ImGui::Text("Timings");
        for(std::pair<std::string, Timespan> timer : Timer::timers)
        {
            std::string text;
            text += timer.first;
            text += ": ";
            text += std::to_string(timer.second.getDuration());
            ImGui::Text(text.c_str());
        }
        ImGui::End();
    }

    Timer::start("Post-Render");
    // Rendering
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
    Timer::stop("Post-Render");
}
ImVec2 Renderer::toImVec2(Vector2 v)
{
    // return ImVec2(
    //     v.x + (windowSize.x / 2.0f),
    //     v.y + (windowSize.y / 2.0f)
    // );

    return ImVec2(v.x, v.y);
}
void Renderer::renderRigidBody(ImDrawList* drawList, const Entity* entity)
{
    ImVec2 windowSize = ImGui::GetWindowSize();
    ImVec2 windowCenter;
    windowCenter.x = windowSize.x / 2;
    windowCenter.y = windowSize.y / 2;

    Vector2 pos = entity->rigidBody->transform.position;

    if(renderDebug)
    {
        AABBCollider aabb = entity->collider->GetAABB(&entity->rigidBody->transform);
        float width = std::abs(aabb.max.x - aabb.min.x);
        float height = std::abs(aabb.max.y - aabb.min.y);
        drawList->AddQuad(
            toImVec2(aabb.min),
            toImVec2(aabb.min + Vector2(width, 0)),
            toImVec2(aabb.max),
            toImVec2(aabb.min + Vector2(0, height)),
            GREEN
        );
    }

    if(dynamic_cast<CircleCollider*>(entity->collider.get()))
    {
        CircleCollider* circleCollider = (CircleCollider*)entity->collider.get();
        float radius = circleCollider->radius;
        float rotation = entity->rigidBody->transform.rotation;
        Vector2 dir = Vector2(radius * std::cos(rotation), radius *std::sin(rotation));

        drawList->AddCircleFilled(toImVec2(pos), radius, WHITE);
        drawList->AddLine(toImVec2(pos), toImVec2(pos + dir), BLACK);
    }

    else if(dynamic_cast<LineCollider*>(entity->collider.get()))
    {
        Transform& transform = entity->rigidBody->transform;
        LineCollider* lineCollider = (LineCollider*)entity->collider.get();

        std::vector<Vector2> points = lineCollider->transformPoints(&transform); 

        drawList->AddLine(toImVec2(points[0]), toImVec2(points[1]), WHITE);
    }
    else if(dynamic_cast<BoxCollider*>(entity->collider.get()))
    {
        Transform& transform = entity->rigidBody->transform;
        BoxCollider* boxCollider = (BoxCollider*) entity->collider.get();

        auto points = boxCollider->transformPoints(&transform);
        drawList->AddQuadFilled(
            toImVec2(points[0]),
            toImVec2(points[1]),
            toImVec2(points[2]),
            toImVec2(points[3]),
            WHITE
        );
    }
    else if(dynamic_cast<ConvexPolygonCollider*>(entity->collider.get()))
    {
        Transform& transform = entity->rigidBody->transform;
        ConvexPolygonCollider* polyCollider = dynamic_cast<ConvexPolygonCollider*>(entity->collider.get());

        std::vector<Vector2> vertices = polyCollider->transformPoints(&transform);
        std::vector<ImVec2> pointsToDraw;

        for(size_t i = 0; i < vertices.size(); i++)
        {
            pointsToDraw.push_back(std::move(toImVec2(vertices[i])));
        };

        drawList->AddConvexPolyFilled(&pointsToDraw[0], vertices.size(), WHITE);
    }
}

void Renderer::destroy()
{
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
}

bool Renderer::running()
{
    return !glfwWindowShouldClose(window);
}

Vector2 Renderer::getMousePosition()
{
    return mousePosition;
}