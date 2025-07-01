#include <algorithm>
#include <format>
#include <iostream>
#include <iomanip>

#include "Application.h"

#include "Ballistics.h"
#include "Curves.h"

using namespace Ui;
namespace
{
    Ballistics::BulletData BulletData;
    Ballistics::EnvironmentData Environment;
    Ballistics::FiringData FiringData;
    std::vector<Ballistics::TrajectoryDataPoint> TrajectoryDataPoints;
    float MaxX = std::numeric_limits<float>::min();
    float MaxY = std::numeric_limits<float>::min();
    float MinX = std::numeric_limits<float>::max();
    float MinY = std::numeric_limits<float>::max();

    PlotPtr TrajectoryPlot;

    void PlotTrajectory()
    {
#if WITH_SDL
        Curve2D Curve;
        for (size_t nQ = 1; nQ < TrajectoryDataPoints.size(); ++nQ)
        {
            Curve.AddPoint(TrajectoryDataPoints[nQ].DistanceX, TrajectoryDataPoints[nQ].DistanceY);
        }
        TrajectoryPlot->AddCurve(Curve);

        TrajectoryPlot->AddLine({Curve.Extents.Min.x, Curve.Extents.Height()/2.0f}, {Curve.Extents.Max.x, Curve.Extents.Height()/2.0f});
        TrajectoryPlot->AddLine({FiringData.ZeroDistance, Curve.Extents.Min.y}, {FiringData.ZeroDistance, Curve.Extents.Max.y});
        TrajectoryPlot->AddLabel("Zero", {FiringData.ZeroDistance+0.1f,Curve.Extents.Max.y});
        
        // // try to fit one tick per 25 meters
        // PointType2d TickVector = { 25.0f, FiringData.Height+0.01f }; // every 25 meters, 1 cm high
        // const int NumTicks = static_cast<int>((DataRangeMax.x - DataRangeMin.x) / TickVector.x);
        // for (int nTick = 0; nTick < NumTicks; nTick++)
        // {
        //     PointType2d ViewportTickVector;
        //     DataPointToViewport(ScaleTransform, NormalizationTransform, TickVector, ViewportTickVector);
        //     const float ThisTickHalfHeight = (nTick & 1) ? 6 : 3; 
        //     
        //     DrawLine(ViewportTickVector.x, ViewportMidY - ThisTickHalfHeight, ViewportTickVector.x, ViewportMidY + ThisTickHalfHeight);
        //     DrawText(std::format("{:}", (nTick+1)*25), { ViewportTickVector.x, ViewportMidY + ThisTickHalfHeight});
        //     TickVector.x += 25.0f;
        // }
        //
        // const PointType2d ZeroPoint = { FiringData.ZeroDistance, FiringData.Height };
        // PointType2d ViewportZeroPoint;
        // DataPointToViewport(ScaleTransform, NormalizationTransform, ZeroPoint, ViewportZeroPoint);
        // DrawText("[Zero]", ViewportZeroPoint);
        // DrawLine(ViewportZeroPoint.x+2, ViewportMin.y, ViewportZeroPoint.x, ViewportMax.y);

        DrawText(std::format("Muzzle velocity is {:.1f}m/s", BulletData.MuzzleVelocityMs), {10.f, 25.f});
        DrawText(std::format("Zero distance is {:.1f}m", FiringData.ZeroDistance), {10.f, 40.f});
        
        AddPlot(TrajectoryPlot);
#else
        std::cout << "T\tX\tY\tVx\n";
        for (const Ballistics::TrajectoryDataPoint& Q : TrajectoryDataPoints)
        {
            const float KineticEnergy = 0.5f * BulletData.GetMassKg() * (Q.VelocityX * Q.VelocityX);
            std::cout << std::fixed << std::setprecision(2) << Q.T << "\t" << Q.DistanceX << "\t" << Q.DistanceY*100.0f << " cm\t" << Q.VelocityX << " m/s (" << (Q.VelocityX * Ballistics::MsToFtS) << " ft/s)\t" << KineticEnergy << " Joules\n";
        }
#endif
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
        FiringData.Height = 10.0f;
        FiringData.ZeroDistance = 200.0f;
        // roughly one inch at 100m etc
        const float ToleranceM = (2.0f * (FiringData.ZeroDistance * 0.01f)) / 100.0f;

        FiringData.ZeroIn(ToleranceM, Environment);

        Ballistics::SolverParams Solver;
        Solver.MaxTime = 10.0f;
        Solver.TimeStep = 0.01f;
        Solver.MaxX = 350.0f;
        
        Ballistics::SolveTrajectoryG7(TrajectoryDataPoints, FiringData, Environment, Solver);

        for (const Ballistics::TrajectoryDataPoint& Q : TrajectoryDataPoints)
        {
            MaxX = std::max(Q.DistanceX, MaxX);
            MaxY = std::max(Q.DistanceY, MaxY);
            MinX = std::min(Q.DistanceX, MinX);
            MinY = std::min(Q.DistanceY, MinY);
        }
        
    }
}

#ifdef WITH_SDL
void AppInit()
{
    TrajectoryPlot = Plot::Create();
    Solve();
    PlotTrajectory();
}

void AppUpdate()
{
    ClearPlots();
    PlotTrajectory();
}
#else
int main(int /*argc*/, char** /*argv*/)
{
    Solve();
    return 0;
}
#endif
