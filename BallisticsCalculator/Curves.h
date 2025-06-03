#pragma once

namespace Curves
{
    struct CatmullRomSegment
    {
        CatmullRomSegment(float P0, float P1, float P2, float P3)
        {
            H0 = 2.0f * P1;
            H1 = (P2 - P0);
            H2 = (2.0f * P0 - 5.0f * P1 + 4.0f * P2 - P3);
            H3 = (-P0 + 3.0f * P1 - 3.0f * P2 + P3);
        }

        float operator()(float t) const
        {
            const float tsq = t * t;
            const float tcb = tsq * t;
            return 0.5f * (H0 + H1 * t + H2 * tsq + H3 * tcb);
        }

    private:
        float H0;
        float H1;
        float H2;
        float H3;
    };
}
