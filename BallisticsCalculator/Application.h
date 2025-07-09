#pragma once
#include "Algebra.h"
#include <functional>
#include <memory>

namespace Application
{
    using OnMouseMoveDelegateType = std::function<void(const Algebra::Vector2D&)>;
    using OnAppUpdateDelegateType = std::function<void()>;
    void SetMouseMoveDelegate(OnMouseMoveDelegateType&& OnMouseMoveDelegate);
    void SetAppUpdateDelegate(OnAppUpdateDelegateType&& OnAppUpdateDelegate);
    bool Init();
    void Run();
}
