#pragma once

#include <functional>

namespace Solver
{
    struct RungeKutta4
    {
        // dY/dt = f(Y,t)
        using dYdtFunc = std::function<float(float, float)>;

        RungeKutta4() = default;
        void Initialize(float InY0, float InH, dYdtFunc&& IndYdtF)
        {
            dYdt = std::move(IndYdtF);
            t = 0.0f;
            Y = Y0 = InY0;
            h = InH;
        }

        void Reset()
        {
            t = 0.0f;
            Y = Y0;
        }

        float Advance()
        {
            const float HalfH = 0.5f * h;
            const float K1 = dYdt(Y, t);
            const float K2 = dYdt(Y + HalfH * K1, t + HalfH);
            const float K3 = dYdt(Y + HalfH * K2, t + HalfH);
            const float K4 = dYdt(Y + h, Y + h * K3);
            t += h;
            return (Y = Y + (h / 6.0f) * (K1 + 2.0f * K2 + 2.0f * K3 + K4));
        }

        dYdtFunc dYdt;
        float Y0 = 0.0f;
        float t = 0.0f;
        float Y = 0.0f;
        float h = 1.0f;
    };
}