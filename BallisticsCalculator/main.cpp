#include <algorithm>
#include <format>
#include <iostream>
#include <iomanip>

#include "Application.h"

#include "Ballistics.h"
#include "Curves.h"

using namespace Plotter;
namespace
{
    Ballistics::BulletData BulletData;
    Ballistics::EnvironmentData Environment;
    Ballistics::FiringData FiringData;
    std::vector<Ballistics::TrajectoryDataPoint> TrajectoryDataPoints;

    PlotPtr TrajectoryPlot;

    void DrawUi()
    {
        if ( !TrajectoryPlot )
        {
            TrajectoryPlot = Plot::Create();
            Curve2D Curve;
            for (size_t nQ = 1; nQ < TrajectoryDataPoints.size(); ++nQ)
            {
                Curve.AddPoint(TrajectoryDataPoints[nQ].DistanceX, TrajectoryDataPoints[nQ].DistanceY);
            }
            Curve.Color = Magenta;
            TrajectoryPlot->AddCurve(Curve);
            const Range2D PlotRange = TrajectoryPlot->GetExtents();
            
            const float CurveHeight = PlotRange.Height();
            const float CenterLineYPos = PlotRange.Min.y + CurveHeight/2.0f;
            const float HeightLineXPos = PlotRange.Min.x + 1.0f;
            TrajectoryPlot->AddLine({PlotRange.Min.x, CenterLineYPos}, {PlotRange.Max.x, CenterLineYPos}, Gray);
            TrajectoryPlot->AddLine({HeightLineXPos, PlotRange.Min.y}, {HeightLineXPos, PlotRange.Max.y}, Gray);
            TrajectoryPlot->AddLine({FiringData.ZeroDistance, PlotRange.Min.y}, {FiringData.ZeroDistance, PlotRange.Max.y}, Gray);
            TrajectoryPlot->AddLabel("Zero", {FiringData.ZeroDistance+0.1f,PlotRange.Max.y}, Blue);

            // try to fit one tick per 25 meters
            Point2D TickVector = { 25.0f, PlotRange.Min.y }; // every 25 meters
            
            int NumTicks = static_cast<int>((PlotRange.Max.x - PlotRange.Min.x) / TickVector.x);
            for (int nTick = 0; nTick < NumTicks; nTick++)
            {
                const float ThisTickHalfHeight = (nTick & 1) ? 0.02f : 0.01f;
                TrajectoryPlot->AddLine({TickVector.x, CenterLineYPos - ThisTickHalfHeight}, {TickVector.x, CenterLineYPos + ThisTickHalfHeight}, Gray);
                TrajectoryPlot->AddLabel(std::format("{:}", (nTick+1)*25), {TickVector.x, CenterLineYPos - 1.1f * ThisTickHalfHeight});
                TickVector.x += 25.0f;
            }

            // vertical tick marks, one every 1/10th of the height
            NumTicks = 10;
            TickVector.x = HeightLineXPos;
            TickVector.y = PlotRange.Min.y;
            const float TickHeight = PlotRange.Height() / static_cast<float>(NumTicks);
            for (int nTick = 0; nTick < NumTicks; nTick++)
            {
                TrajectoryPlot->AddLine({TickVector.x, TickVector.y}, {TickVector.x + 1.0f, TickVector.y},Gray);
                TrajectoryPlot->AddLabel(std::format("{:.1f}", TickVector.y), {TickVector.x + 1.1f, TickVector.y});
                TickVector.y += TickHeight;
            }
        }
        
        DrawText(std::format("Muzzle velocity is {:.1f}m/s", BulletData.MuzzleVelocityMs), {10.f, 25.f});
        DrawText(std::format("Zero distance is {:.1f}m", FiringData.ZeroDistance), {10.f, 40.f});
        Range2D ViewportExtents = GetRenderer()->GetViewportExtents();
        ViewportExtents.Min.x += ViewportExtents.Width()*0.1f;
        ViewportExtents.Min.y += ViewportExtents.Height()*0.15f;
        ViewportExtents.Max.x -= ViewportExtents.Width()*0.1f;
        ViewportExtents.Max.y -= ViewportExtents.Height()*0.15f;
        DrawPlot(TrajectoryPlot, ViewportExtents);
    }

    void Solve()
    {
        BulletData.MassGr = 155.0f;
        BulletData.G7BC = 0.275f;
        BulletData.MuzzleVelocityMs = 871.42f;
        BulletData.CallibreMm = Ballistics::Callibre308Mm;

        Environment.Gravity = -9.81f;
        Environment.TKelvin = 292.0f;
        Environment.AirPressure = 101325.0f;
        Environment.UpdateAirDensityFromTandP();
    
        FiringData.Bullet = BulletData;
        FiringData.Height = 1.0f;
        FiringData.ZeroDistance = 200.0f;
        // roughly one inch at 100m etc
        const float ToleranceM = (2.0f * (FiringData.ZeroDistance * 0.01f)) / 100.0f;

        FiringData.ZeroIn(ToleranceM, Environment);

        Ballistics::SolverParams Solver;
        Solver.MaxTime = 10.0f;
        Solver.TimeStep = 0.01f;
        Solver.MaxX = 350.0f;
        
        Ballistics::SolveTrajectoryG7(TrajectoryDataPoints, FiringData, Environment, Solver);
    }

    const Ballistics::TrajectoryDataPoint& NearestPoint(const Point2D& SamplePoint)
    {
        float MinDistance = std::numeric_limits<float>::max();
        size_t MinIndex = 0;
        for (size_t nQ = 1; nQ < TrajectoryDataPoints.size(); ++nQ)
        {
            const float DeltaX = TrajectoryDataPoints[nQ].DistanceX - SamplePoint.x;
            const float DeltaY = TrajectoryDataPoints[nQ].DistanceY - SamplePoint.y;
            const float DeltaSq = (DeltaX * DeltaX) + (DeltaY * DeltaY);
            if (DeltaSq < MinDistance)
            {
                MinDistance = DeltaSq;
                MinIndex = nQ;
            }
        }
        return TrajectoryDataPoints[MinIndex];
    }
    
}

#ifdef WITH_SDL
void AppInit()
{
    Solve();
}

void AppUpdate()
{
    DrawUi();
}
#else
int main(int /*argc*/, char** /*argv*/)
{
    Solve();
    return 0;
}
#endif
