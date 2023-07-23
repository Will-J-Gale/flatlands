#pragma once

#include <memory>
#include <vector>
#include <functional>
#include <windows/Window.h>

enum class Demo
{
    None=0,
    PhysicsDemo,
    CircleCircle,
    CapsuleCircle,
    CapsuleCapsule,
    CirclePolygon,
    CapsulePolygon
};

typedef std::function<void(Demo)> OnDemoSelectedCallback;

class DemoSelectWindow : public Window
{
public:
    DemoSelectWindow();
    void SetDemoSelectCallback(OnDemoSelectedCallback demoSelectedCallback);
    void Render() override;

private:
    OnDemoSelectedCallback demoSelectedCallback = nullptr;
};

