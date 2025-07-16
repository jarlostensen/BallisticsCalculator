#pragma once
#include "Algebra.h"
#include <functional>
#include <memory>

namespace Application
{
    using OnMouseMoveDelegateType = std::function<void(const Algebra::Vector2D&)>;
    using OnMouseButtonDelegateType = std::function<void(bool bPressed, const Algebra::Vector2D&)>;
    using OnAppUpdateDelegateType = std::function<void()>;
    void SetMouseMoveDelegate(OnMouseMoveDelegateType&& OnMouseMoveDelegate);
    void SetAppUpdateDelegate(OnAppUpdateDelegateType&& OnAppUpdateDelegate);
    void SetMouseButtonDelegate(OnMouseButtonDelegateType&& OnMouseButtonDelegate);
    bool Init();
    void Run();
}
