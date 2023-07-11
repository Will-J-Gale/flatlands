#include <stdio.h>
#include <Renderer.h>

#include <core/Logger.h>
#include <collision/CollisionAlgorithms.h>
#include <Constants.h>
#include <core/Timer.h>
#include <core/Metrics.h>
#include <collision/broadPhase/QuadTree.h>

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

// void Renderer::drawCollisionDetection(ImDrawList* drawList, std::vector<std::shared_ptr<Entity>>& entities)
// {
//     //Draw circle/polygon axis projections
//     Entity* entity =  entities[0].get();
//     Entity* circle = entities[1].get();

//     ConvexPolygonCollider* polyCollider = dynamic_cast<ConvexPolygonCollider*>(entity->collider.get());
//     Transform* polyTransform = &entity->rigidBody->transform;

//     CircleCollider* circleCollider = dynamic_cast<CircleCollider*>(circle->collider.get());
//     Transform* circleTransform = &circle->rigidBody->transform;

//     // for(Vector2& axis : polyCollider->getAxes(transform.rotation))
//     // {
//     //     float axisLength = 1000.0f;
//     //     Vector2 positiveEnd = axis * axisLength; 
//     //     Vector2 negativeEnd = -axis * axisLength; 
//     //     drawList->AddLine(toImVec2(Vector2()), toImVec2(positiveEnd), BLUE, 2.0f);
//     //     drawList->AddLine(toImVec2(Vector2()), toImVec2(negativeEnd), WHITE, 2.0f);
        
//     //     auto polygonProjection = CollisionAlgorithms::projectShapeOntoAxis(axis, polyCollider->transformPoints(&transform));
//     //     drawList->AddCircleFilled(toImVec2(axis * polygonProjection.min), 4.0f, GREEN);
//     //     drawList->AddCircleFilled(toImVec2(axis * polygonProjection.max), 7.0f, GREEN);

//     //     auto circleProjection = CollisionAlgorithms::projectCircleOnToAxis(circleCollider->radius, circleTransform.position, axis);
//     //     drawList->AddCircleFilled(toImVec2(axis * circleProjection.min), 4.0f, RED);
//     //     drawList->AddCircleFilled(toImVec2(axis * circleProjection.max), 7.0f, RED);
//     // }

//     //SAT 2.0
//     //Algorithm is a little wonky because Y is flipped from most game engines (Normal up is positive, this down is positive)
//     CollisionPoints collisionPoints;
//     collisionPoints.depth = Math::FLOAT_MAX;

//     //Get shape vertices
//     std::vector<Vector2> aPoints = polyCollider->transformPoints(polyTransform);
//     std::vector<Vector2> aAxes = polyCollider->getAxes(polyTransform->rotation);

//     for(Vector2& axis : aAxes)
//     {
//         auto aProjection = CollisionAlgorithms::projectShapeOntoAxis(axis, aPoints);
//         auto bProjection = CollisionAlgorithms::projectCircleOnToAxis(circleCollider->radius, circleTransform->position, axis);

//         Vector2 box_projection = polyTransform->position + (axis * aProjection.min);

//         if(aProjection.min >= bProjection.max || bProjection.min >= aProjection.max)
//         {
//             collisionPoints.hasCollisions = false;
//             // return;
//         }
        
//         // drawAxis(drawList, axis);
//         // drawPolygonOnAxis(drawList, axis, aPoints);
//         // drawCircleOnAxis(drawList, axis, circleCollider->radius, circleTransform->position);

//         float axisDepth = std::min(bProjection.max - aProjection.min, aProjection.max - bProjection.min);

//         if(axisDepth < collisionPoints.depth)
//         {
//             collisionPoints.depth = axisDepth;
//             collisionPoints.normal = axis;
//         }
//     }

//     Vector2 closestVertex = CollisionAlgorithms::getClosestVertexToPoint(circleTransform->position, aPoints);
//     Vector2 axis = (closestVertex - circleTransform->position).normalize();

//     drawList->AddCircleFilled(toImVec2(closestVertex), 10.0f, CYAN);
//     auto aProjection = CollisionAlgorithms::projectShapeOntoAxis(axis, aPoints);
//     auto bProjection = CollisionAlgorithms::projectCircleOnToAxis(circleCollider->radius, circleTransform->position, axis);

//     drawAxis(drawList, axis);
//     drawPolygonOnAxis(drawList, axis, aPoints);
//     drawCircleOnAxis(drawList, axis, circleCollider->radius, circleTransform->position);
//     drawList->AddLine(toImVec2(closestVertex), toImVec2(circleTransform->position), GREEN);

//     if(aProjection.min >= bProjection.max || bProjection.min >= aProjection.max)
//     {
//         collisionPoints.hasCollisions = false;
//         return;
//     }

//     float axisDepth = std::min(bProjection.max - aProjection.min, aProjection.max - bProjection.min);

//     if(axisDepth < collisionPoints.depth)
//     {
//         collisionPoints.depth = axisDepth;
//         collisionPoints.normal = axis;
//     }

//     Vector2 centerA = CollisionAlgorithms::getCenterPoint(aPoints);
//     Vector2 dir = circleTransform->position - centerA;

//     if(Vector2::dot(dir, collisionPoints.normal) < 0)
//         collisionPoints.normal *= -1;

// }


void Renderer::drawCollisionDetection(ImDrawList* drawList, std::vector<std::shared_ptr<Entity>>& entities)
{
    // CapsuleCollider* b = dynamic_cast<CapsuleCollider*>(entities[0]->collider.get());
    // Transform* bTransform = &entities[0]->rigidBody->transform;
    // CircleCollider* a = dynamic_cast<CircleCollider*>(entities[1]->collider.get());
    // Transform* aTransform = &entities[1]->rigidBody->transform;

    // float capsuleRadius = b->GetWidth() / 2.0f;
    // Line capsuleCenterLine = b->GetCenterLine(bTransform);
    // Vector2 closestPoint = capsuleCenterLine.closestPointOnLine(aTransform->position);
    // float distanceToCircle = Vector2::distance(closestPoint, aTransform->position);
    // float radiusSum = capsuleRadius + a->radius;

    // if(distanceToCircle <= radiusSum)
    // {
    //     Vector2 direction = aTransform->position - closestPoint;
    //     Vector2 contact = aTransform->position + (-direction.normalize() * a->radius);
    //     float depth = radiusSum - direction.magnitude();
        
    //     // Vector2 normal = direction.normalize();
    //     Vector2 end = contact + (direction.normalize() * depth);
    //     drawList->AddLine(toImVec2(contact), toImVec2(end), RED, 2.0f);
    //     drawList->AddCircleFilled(toImVec2(contact), 3.0f, RED);

    //     Transform newTransform = *bTransform;
    //     newTransform.position += (-direction.normalize() * depth);
    //     drawCapsule(drawList, b, &newTransform, MAGENTA);
    // }
    
    // drawList->AddLine(toImVec2(aTransform->position), toImVec2(closestPoint), GREEN);
    // drawList->AddCircleFilled(toImVec2(closestPoint), 5.0f, CYAN);
}

void Renderer::drawCapsuleCapsuleCollisionTest(ImDrawList* drawList, CapsuleCollider* a, Transform* aTransform, CapsuleCollider* b, Transform* bTransform)
{
    Line aLine = a->GetCenterLine(aTransform);
    Line bLine = b->GetCenterLine(bTransform);

    drawList->AddLine(toImVec2(aLine.start), toImVec2(aLine.end), BLACK);
    drawList->AddLine(toImVec2(bLine.start), toImVec2(bLine.end), BLACK);

    ClosestVertexProjection closestVertexProjectionOnA = aLine.closestVertexOnLine({bLine.start, bLine.end});
    ClosestVertexProjection closestVertexProjectionOnB = bLine.closestVertexOnLine({aLine.start, aLine.end});
    ClosestVertexProjection closestProjection = closestVertexProjectionOnA;
    bool vertexIsA = false;

    if(closestVertexProjectionOnB.distance < closestVertexProjectionOnA.distance)
    {
        closestProjection = closestVertexProjectionOnB;
        vertexIsA = true;
    }
    
    drawList->AddCircleFilled(toImVec2(closestProjection.vertex), 3.0f, MAGENTA);
    drawList->AddLine(toImVec2(closestProjection.projectedPoint), toImVec2(closestProjection.vertex), MAGENTA, 2.0f);

    float aRadius = a->GetWidth() / 2.0f;
    float bRadius = b->GetWidth() / 2.0f;
    float radiusSum = aRadius + bRadius;

    if(vertexIsA)
    {
        drawList->AddCircle(toImVec2(closestProjection.vertex), aRadius, RED);
        drawList->AddCircle(toImVec2(closestProjection.projectedPoint), bRadius, RED);
    }
    else
    {
        drawList->AddCircle(toImVec2(closestProjection.vertex), bRadius, RED);
        drawList->AddCircle(toImVec2(closestProjection.projectedPoint), aRadius, RED);
    }

    if(closestProjection.distance < radiusSum)
    {
        Vector2 normal = (closestProjection.projectedPoint - closestProjection.vertex).normalize();

        float depth = radiusSum - closestProjection.distance;
        Transform translated = *aTransform;
        translated.position -= normal * depth;

        drawCapsule(drawList, a, &translated, BLUE);

        float radiusOfVertex = vertexIsA ? aRadius : bRadius;
        Vector2 dir = (closestProjection.projectedPoint - closestProjection.vertex).normalize();
        Vector2 contactPoint = closestProjection.vertex + (dir * radiusOfVertex);

        drawList->AddCircleFilled(toImVec2(contactPoint), 5.0f, GREEN);
    }
}

void Renderer::drawCircleCapsuleCollisionTest(ImDrawList* drawList, CircleCollider* a, Transform* aTransform, CapsuleCollider* b, Transform* bTransform)
{
    float capsuleRadius = b->GetWidth() / 2.0f;
    Line capsuleCenterLine = b->GetCenterLine(bTransform);
    Vector2 closestPoint = capsuleCenterLine.closestPointOnLine(aTransform->position);
    float distanceToCircle = Vector2::distance(closestPoint, aTransform->position);
    float radiusSum = capsuleRadius + a->radius;

    if(distanceToCircle <= radiusSum)
    {
        Vector2 direction = aTransform->position - closestPoint;
        Vector2 contact = aTransform->position + (-direction.normalize() * a->radius);
        float depth = radiusSum - direction.magnitude();
        
        // Vector2 normal = direction.normalize();
        Vector2 end = contact + (direction.normalize() * depth);
        drawList->AddLine(toImVec2(contact), toImVec2(end), RED, 2.0f);
        drawList->AddCircleFilled(toImVec2(contact), 3.0f, RED);

        Transform newTransform = *bTransform;
        newTransform.position += (-direction.normalize() * depth);
        drawCapsule(drawList, b, &newTransform, MAGENTA);
    }
    
    drawList->AddLine(toImVec2(aTransform->position), toImVec2(closestPoint), GREEN);
    drawList->AddCircleFilled(toImVec2(closestPoint), 5.0f, CYAN);
}

void Renderer::drawCapsulePolygonCollision(ImDrawList* drawList, CapsuleCollider* a, Transform* aTransform, ConvexPolygonCollider* b, Transform* bTransform)
{
    float aRadius = a->GetWidth() / 2.0f;
    Line centerLine = a->GetCenterLine(aTransform);
    std::vector<Vector2> centerLineVertices = {centerLine.start, centerLine.end};
    std::vector<Vector2> bPoints = b->transformPoints(bTransform);
    std::vector<Line> bEdges = b->getEdges(bTransform);

    ClosestVertexProjection closestProjection = centerLine.closestVertexOnLine(bPoints);
    Line closestEdge = bEdges[0];
    float closestEdgeDistance = Math::FLOAT_MAX;

    for(Line& edge : bEdges)
    {
        ClosestVertexProjection projection = edge.closestVertexOnLine(centerLineVertices);

        if(projection.distance < closestProjection.distance)
            closestProjection = projection;

        Vector2 edgeCenter = edge.start + ((edge.end - edge.start) / 2.0f);
        float edgeDistance = Vector2::distanceSquared(aTransform->position,  edgeCenter);

        if(edgeDistance < closestEdgeDistance)
        {
            closestEdge = edge;
            closestEdgeDistance = edgeDistance;
        }
    }

    Vector2 capsuleLineDir = (centerLine.end - centerLine.start).normalize();
    Vector2 edgeDir = (closestEdge.end - closestEdge.start).normalize();

    drawList->AddCircleFilled(toImVec2(closestProjection.projectedPoint), 3.0f, GREEN);
    drawList->AddLine(toImVec2(closestProjection.vertex), toImVec2(closestProjection.projectedPoint), GREEN);
    drawList->AddLine(toImVec2(closestEdge.start), toImVec2(closestEdge.end), GREEN);

    if(closestProjection.distance < aRadius)
    {
        Vector2 normal = (closestProjection.projectedPoint - closestProjection.vertex).normalize();
        float depth = aRadius - closestProjection.distance;

        Vector2 bodyDir = bTransform->position - aTransform->position;
        if(Vector2::dot(bodyDir, normal) < 0)
            normal *= -1;

        Transform capsuleTransform = *aTransform;
        capsuleTransform.position -= normal * depth;
        drawCapsule(drawList, a, &capsuleTransform, BLUE);
    }  
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

Renderer::~Renderer()
{
    destroy();
}

void Renderer::render(std::vector<std::shared_ptr<Entity>>& entities, std::vector<Collision>* collisions, Metrics worldMetrics)
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

        float halfWidth = windowSize.x / 2.0f;
        float halfHeight = windowSize.y / 2.0f;
        AABB boundary = AABB(Vector2(0, 0), Vector2(windowSize.x, windowSize.y));
        QuadTree quadTree = QuadTree(boundary, 4);

        for(std::shared_ptr<Entity>& entity : entities)
        {
            renderRigidBody(drawList, entity.get());
            quadTree.Insert(entity->rigidBody->transform.position);
        }

        auto boundsList = std::vector<AABB>();
        quadTree.GetTreeBounds(&boundsList);

        for(AABB& bounds : boundsList)
        {
            auto p1 = bounds.min;
            auto p2 = Vector2(bounds.min.x + bounds.width, bounds.min.y);
            auto p3 = bounds.max;
            auto p4 = Vector2(bounds.min.x, bounds.min.y + bounds.height);

            drawList->AddQuad(
                toImVec2(p1),
                toImVec2(p2),
                toImVec2(p3),
                toImVec2(p4), 
                WHITE
            );
        }

        float queryWidth = 50;
        float queryHeight = 50;

        AABB queryBox = AABB(
            Vector2(mousePosition.x - queryWidth, mousePosition.y - queryHeight),
            Vector2(mousePosition.x + queryWidth, mousePosition.y + queryHeight)
        );

        Vector2 p1(mousePosition.x - queryWidth, mousePosition.y - queryWidth);
        drawList->AddQuad(
            toImVec2(Vector2(mousePosition.x - queryWidth, mousePosition.y - queryWidth)),
            toImVec2(Vector2(mousePosition.x + queryWidth, mousePosition.y - queryWidth)),
            toImVec2(Vector2(mousePosition.x + queryWidth, mousePosition.y + queryWidth)),
            toImVec2(Vector2(mousePosition.x - queryWidth, mousePosition.y + queryWidth)),
            GREEN
        );

        std::vector<Vector2> points;
        quadTree.Query(queryBox, &points);

        for(Vector2& point : points)
        {
            drawList->AddCircleFilled(toImVec2(point), 10.0f, GREEN);
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
                        drawList->AddCircleFilled(toImVec2(contact), 10.0f, RED);

                        Vector2 frictionImpulse = collision.collisionPoints.frictionImpulses[i];
                        Vector2 impulseEnd = contact + (frictionImpulse * 1);
                        drawList->AddLine(toImVec2(contact), toImVec2(impulseEnd), MAGENTA, 5.0f);
                    }
                }
            }
        }

        // drawCollisionDetection(drawList, entities);

        // auto a = entities[0];
        // auto b = entities[1];
        // drawCapsuleCapsuleCollisionTest(drawList, (CapsuleCollider*)a->collider.get(), &a->rigidBody->transform, (CapsuleCollider*)b->collider.get(), &b->rigidBody->transform);

        // drawCapsulePolygonCollision(drawList, (CapsuleCollider*)a->collider.get(), &a->rigidBody->transform, (ConvexPolygonCollider*)b->collider.get(), &b->rigidBody->transform);

        // auto a2 = entities[3];
        // auto b2 = entities[2];
        // drawCapsulePolygonCollision(drawList, (CapsuleCollider*)a2->collider.get(), &a2->rigidBody->transform, (ConvexPolygonCollider*)b2->collider.get(), &b2->rigidBody->transform);

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

        ImGui::Separator();
        ImGui::Text("World Metrics");
        ImGui::Text(std::string("\tEntities: " + std::to_string(worldMetrics.numEntities)).c_str());
        ImGui::Text(std::string("\tBroad phase checks: " + std::to_string(worldMetrics.broadPhaseChecks)).c_str());
        ImGui::Text(std::string("\tNarrow phase checks: " + std::to_string(worldMetrics.narrowPhaseChecks)).c_str());
        ImGui::Spacing();
        ImGui::Text(std::string("\tStep time: " + std::to_string(worldMetrics.worldStepTime)).c_str());
        ImGui::Text(std::string("\tMove bodies: " + std::to_string(worldMetrics.moveBodiesTime)).c_str());
        ImGui::Text(std::string("\tBroad phase time: " + std::to_string(worldMetrics.broadPhaseTime)).c_str());
        ImGui::Text(std::string("\tNarrow phase time: " + std::to_string(worldMetrics.narrowPhaseTime)).c_str());
        ImGui::Text(std::string("\tResolve collisions: " + std::to_string(worldMetrics.resolveCollisionsTime)).c_str());
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
    RigidBody* rigidBody = entity->rigidBody.get();
    Collider* collider = entity->rigidBody->collider;
    Transform* transform = &entity->rigidBody->transform;
    Vector2 pos = transform->position;
    float rotation = transform->rotation;
    ImU32 bodyColour = rigidBody->isAwake ? WHITE : WHITE_A;

    if(renderDebug)
    {
        AABB aabb = entity->collider->GetAABB(&entity->rigidBody->transform);
        drawAABB(drawList, &aabb, GREEN);
    }

    if(dynamic_cast<CircleCollider*>(collider))
    {
        CircleCollider* circleCollider = (CircleCollider*)collider;
        float radius = circleCollider->radius;
        Vector2 dir = Vector2(radius * std::cos(rotation), radius *std::sin(rotation));

        drawList->AddCircleFilled(toImVec2(pos), radius, bodyColour);
        drawList->AddLine(toImVec2(pos), toImVec2(pos + dir), BLACK);
    }

    else if(dynamic_cast<LineCollider*>(collider))
    {
        LineCollider* lineCollider = (LineCollider*)collider;
        std::vector<Vector2> points = lineCollider->transformPoints(transform); 

        drawList->AddLine(toImVec2(points[0]), toImVec2(points[1]), bodyColour);
    }
    else if(dynamic_cast<BoxCollider*>(collider))
    {
        BoxCollider* boxCollider = (BoxCollider*)collider;
        drawBox(drawList, boxCollider, transform, bodyColour);
    }
    else if(dynamic_cast<LineCollider*>(collider))
    {
        LineCollider* lineCollider = dynamic_cast<LineCollider*>(collider);
        Line line = lineCollider->transformLine(transform);
        drawList->AddLine(toImVec2(line.start), toImVec2(line.end), bodyColour, 2.0f);
    }
    else if(dynamic_cast<ConvexPolygonCollider*>(collider))
    {
        ConvexPolygonCollider* polyCollider = dynamic_cast<ConvexPolygonCollider*>(collider);

        std::vector<Vector2> vertices = polyCollider->transformPoints(transform);
        std::vector<ImVec2> pointsToDraw;

        for(size_t i = 0; i < vertices.size(); i++)
        {
            pointsToDraw.push_back(std::move(toImVec2(vertices[i])));
        };

        drawList->AddConvexPolyFilled(&pointsToDraw[0], vertices.size(), bodyColour);
    }
    else if(dynamic_cast<CapsuleCollider*>(collider))
    {
        CapsuleCollider* capsuleCollider = dynamic_cast<CapsuleCollider*>(collider);

        drawCapsule(drawList, capsuleCollider, transform, bodyColour);
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

void Renderer::drawCapsule(ImDrawList* drawList, CapsuleCollider* capsule, Transform* transform, ImU32 colour)
{
    Vector2 position = transform->position;
    float radius = capsule->GetWidth() / 2.0f;
    float height = capsule->GetHeight();
    float halfHeight = height / 2.0f;
    Line centerLine = capsule->GetCenterLine(transform);

    drawList->AddCircleFilled(toImVec2(centerLine.start), radius, colour);
    drawList->AddCircleFilled(toImVec2(centerLine.end), radius, colour);

    float boxHeight = halfHeight - radius;

    drawList->AddQuadFilled(
        toImVec2(Vector2(-radius, -boxHeight).rotate(transform->rotation) + transform->position),
        toImVec2(Vector2(radius, -boxHeight).rotate(transform->rotation) + transform->position),
        toImVec2(Vector2(radius, boxHeight).rotate(transform->rotation) + transform->position),
        toImVec2(Vector2(-radius, boxHeight).rotate(transform->rotation) + transform->position),
        colour
    );
}

void Renderer::drawBox(ImDrawList* drawList, BoxCollider* box, Transform* transform, ImU32 colour)
{
    auto points = box->transformPoints(transform);
    drawList->AddQuadFilled(
        toImVec2(points[0]),
        toImVec2(points[1]),
        toImVec2(points[2]),
        toImVec2(points[3]),
        colour
    );
}

void Renderer::drawAABB(ImDrawList* drawList, AABB* aabb, ImU32 colour)
{
    float width = std::abs(aabb->max.x - aabb->min.x);
    float height = std::abs(aabb->max.y - aabb->min.y);
    drawList->AddQuad(
        toImVec2(aabb->min),
        toImVec2(aabb->min + Vector2(width, 0)),
        toImVec2(aabb->max),
        toImVec2(aabb->min + Vector2(0, height)),
        GREEN
    );
}