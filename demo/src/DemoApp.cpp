#include <iostream>
#include <vector>
#include <cstdlib>
#include <thread>

#include <imgui.h>

#include "DemoApp.h"
#include <core/Time.h>
#include <core/Logger.h>
#include <core/Timer.h>
#include <windows/collisionDetection/CapsuleCapsuleCollisionDetectionWindow.h>
#include <windows/collisionDetection/CapsulePolygonCollisionDetectionWindow.h>
#include <windows/collisionDetection/CircleCapsuleCollisionDetectionWindow.h>
#include <windows/collisionDetection/CircleCircleCollisionDetectionWindow.h>
#include <windows/collisionDetection/CirclePolygonCollisionDetectionWindow.h>


inline static void glfw_error_callback(int error, const char *description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}


DemoApp::DemoApp()
{
    demoSelectWindow.SetDemoSelectCallback(std::bind(&DemoApp::OnDemoSelected, this, std::placeholders::_1));

    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return;

    const char *glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    window = glfwCreateWindow(1920, 1080, "Flatlands", NULL, NULL);

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    // ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
}

void DemoApp::Destroy()
{
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
}

void DemoApp::Run()
{
    while(!glfwWindowShouldClose(window))
    {
        Timer::start("Render");
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
        
        demoSelectWindow.Render();

        if(demoWindow != nullptr)
            demoWindow->Render();

        // Rendering
        ImGui::Render();
        Timer::start("Render");

        Timer::start("Post-Render");
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        Timer::stop("Post-Render");
    }
}

void DemoApp::OnDemoSelected(Demo demo)
{
    if(demo == Demo::PhysicsDemo)
        demoWindow = std::make_unique<PhysicsDemoWindow>();
    else if(demo == Demo::CircleCircle)
        demoWindow = std::make_unique<CircleCircleCollisionDetectionWindow>();
    else if(demo == Demo::CirclePolygon)
        demoWindow = std::make_unique<CirclePolygonCollisionDetectionWindow>();
    else if(demo == Demo::CapsulePolygon)
        demoWindow = std::make_unique<CapsulePolygonCollisionDetectionWindow>();
    else if(demo == Demo::CapsuleCircle)
        demoWindow = std::make_unique<CircleCapsuleCollisionDetectionWindow>();
    else if(demo == Demo::CapsuleCapsule)
        demoWindow = std::make_unique<CapsuleCapsuleCollisionDetectionWindow>();
}