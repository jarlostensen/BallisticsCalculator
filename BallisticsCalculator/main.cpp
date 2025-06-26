#include <algorithm>
#include <iostream>
#include <iomanip>

#include "Application.h"

#include "BallisticsCalculator.h"
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

    void DrawTrajectory(bool ClearCurrent)
    {
#if WITH_SDL
        if (ClearCurrent)
        {
            ClearCurves();   
        }
        Curve2D Curve;
        for (size_t nQ = 1; nQ < TrajectoryDataPoints.size(); ++nQ)
        {
            Curve.AddPoint(TrajectoryDataPoints[nQ].DistanceX, TrajectoryDataPoints[nQ].DistanceY);    
        }
        PlotCurve(Curve);
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
        FiringData.ZeroDistance = 100.0f;
        FiringData.ZeroIn(0.01f, Environment);

        Ballistics::SolverParams Solver;
        Solver.MaxTime = 10.0f;
        Solver.TimeStep = 0.01f;
        Solver.MaxX = 200.0f;
        
        Ballistics::SolveTrajectoryG7(TrajectoryDataPoints, FiringData, Environment, Solver);

        for (const Ballistics::TrajectoryDataPoint& Q : TrajectoryDataPoints)
        {
            MaxX = std::max(Q.DistanceX, MaxX);
            MaxY = std::max(Q.DistanceY, MaxY);
            MinX = std::min(Q.DistanceX, MinX);
            MinY = std::min(Q.DistanceY, MinY);
        }
        DrawTrajectory(false);

        PlotText("Ballistics calculator", {10.f, 25.f});
    }
}

#ifdef WITH_SDL
void AppInit()
{
    Solve();
}
void AppUpdate()
{
    DrawTrajectory(true);
}
#else
int main(int /*argc*/, char** /*argv*/)
{
    Solve();
    return 0;
}
#endif
