#pragma once
#include <map>

namespace Ballistics
{
    using DragTableType = std::map<float, float>;
    extern const DragTableType G7;

    float GetDragCoefficient(const DragTableType& Table, float Speed, float TemperatureK);
}