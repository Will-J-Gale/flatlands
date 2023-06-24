#include <stdio.h>
#include <Renderer.h>
#include <colliders/CircleCollider.h>
#include <colliders/LineCollider.h>
#include <colliders/BoxCollider.h>
#include <core/Logger.h>
#include <colliders/CollisionAlgorithms.h>

inline static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

Renderer::Renderer()
{
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return;

    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    window = glfwCreateWindow(1920, 1080, "Dear ImGui GLFW+OpenGL3 example", NULL, NULL);
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
    bool show_demo_window = false;
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

    // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
    {
        ImGui::Begin("FPS");                          // Create a window called "Hello, world!" and append into it.

        ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::End();
    }

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

        ImGui::End();
    }

    // Rendering
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
}
ImVec2 Renderer::toImVec2(Vector2 v)
{
    return ImVec2(v.x, v.y);
}
void Renderer::renderRigidBody(ImDrawList* drawList, const Entity* entity)
{
    ImVec2 windowSize = ImGui::GetWindowSize();
    ImVec2 windowCenter;
    windowCenter.x = windowSize.x / 2;
    windowCenter.y = windowSize.y / 2;

    Vector2 pos = entity->rigidBody->transform.position;

    AABBCollider aabb = entity->collider->GetAABB(&entity->rigidBody->transform);
    float width = std::abs(aabb.max.x - aabb.min.x);
    float height = std::abs(aabb.max.y - aabb.min.y);
    drawList->AddQuad(
        toImVec2(aabb.min),
        toImVec2(aabb.min + Vector2(width, 0)),
        toImVec2(aabb.max),
        toImVec2(aabb.min + Vector2(0, height)),
        ImGui::ColorConvertFloat4ToU32(ImVec4(0.0f, 1.0f, 0.0f, 1.0f))
    );

    if(dynamic_cast<CircleCollider*>(entity->collider.get()))
    {
        CircleCollider* circleCollider = (CircleCollider*)entity->collider.get();
        float radius = circleCollider->radius;
        // ImVec2 windowPos = ImGui::GetWindowPos();

        // pos.x += windowPos.x;
        // pos.y += windowPos.y;

        drawList->AddCircleFilled(toImVec2(pos), radius, ImGui::GetColorU32(ImVec4(1.0, 1.0, 1.0, 1.0)));
    }

    else if(dynamic_cast<LineCollider*>(entity->collider.get()))
    {
        Transform& transform = entity->rigidBody->transform;
        LineCollider* lineCollider = (LineCollider*)entity->collider.get();

        std::vector<Vector2> points = lineCollider->transformPoints(transform.position, transform.rotation); 

        drawList->AddLine(toImVec2(points[0]), toImVec2(points[1]), ImGui::ColorConvertFloat4ToU32(ImVec4(1.0f, 1.0f, 1.0f, 1.0f)));
    }
    else if(dynamic_cast<BoxCollider*>(entity->collider.get()))
    {
        Transform& transform = entity->rigidBody->transform;
        BoxCollider* boxCollider = (BoxCollider*) entity->collider.get();

        auto points = boxCollider->transformPoints(transform.position, transform.rotation);
        drawList->AddQuadFilled(
            toImVec2(points[0]),
            toImVec2(points[1]),
            toImVec2(points[2]),
            toImVec2(points[3]),
            ImGui::ColorConvertFloat4ToU32(ImVec4(1.0f, 1.0f, 1.0f, 1.0f))
        );
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