#pragma once

#include <vector>
#include "Algebra.h"
#include "Plotter.h"

namespace Curves
{
    /**
     * @brief Represents a single segment of a Catmull-Rom spline.
     *
     * The Catmull-Rom spline segment is a type of cubic Hermite spline used for interpolating between
     * points, providing a smooth curve that passes through all control points.
     */
    template<typename T>
    struct TCatmullRomSegment
    {
        TCatmullRomSegment(T P0, T P1, T P2, T P3)
        {
            H0 = 2.0f * P1;
            H1 = (P2 - P0);
            H2 = (2.0f * P0 - 5.0f * P1 + 4.0f * P2 - P3);
            H3 = (-P0 + 3.0f * P1 - 3.0f * P2 + P3);
        }

        T operator()(float t) const
        {
            const float tsq = t * t;
            const float tcb = tsq * t;
            return 0.5f * (H0 + H1 * t + H2 * tsq + H3 * tcb);
        }

        /**
         * Adaptively samples the curve between t and t+dt to within error and returns *pairs* of points in the provided vector
         * @param OutSamples 
         * @param t 
         * @param dt 
         * @param error 
         */
        void SampleAdaptively(std::vector<T>& OutSamples, float t, float dt, float error) const
        {
            const T S0 = this->operator()(t);
            const T S1 = this->operator()(t+dt);
            SampleAdaptivelyImpl(OutSamples, S0,S1, t, t+dt, error);
        }

    private:
        T H0;
        T H1;
        T H2;
        T H3;

        void SampleAdaptivelyImpl(std::vector<T>& OutSamples, T S0, T S1, float T0, float T1, float Error) const
        {
            const float TMid = (T0+T1)/2.0f;
            const T SMid = this->operator()(TMid);
            const T LinearMidPt = 0.5f*(S0+S1);
            if ( fabsf(SMid-LinearMidPt) <= Error )
            {
                OutSamples.push_back(S0);
                OutSamples.push_back(S1);
            }
            else
            {
                SampleAdaptivelyImpl(OutSamples, S0, SMid, T0, TMid, Error);
                SampleAdaptivelyImpl(OutSamples, SMid, S1, TMid, T1, Error);
            }
        }
    };
    using CatmullRomSegment1D = TCatmullRomSegment<float>;
    using CatmullRomSegment2D = TCatmullRomSegment<Algebra::Vector2D>;
}
