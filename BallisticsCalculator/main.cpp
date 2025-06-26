#include <algorithm>
#include <format>
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

    void DrawGrid()
    {
        const Range2D MaximalDataRange = GetPlotRange();
        if (MaximalDataRange.IsNonEmpty())
        {
            ClearLines();
            ClearText();
            
            PointType2d ScaleTransform;
            PointType2d NormalizationTransform;
            GenerateTransforms(ScaleTransform, NormalizationTransform);

            const PointType2d DataRangeMin = MaximalDataRange.Min;
            const PointType2d DataRangeMax = MaximalDataRange.Max;
            constexpr PointType2d TickVector = {25.0f, 0.01f}; // every 25 meters, 1 cm high
            PointType2d ViewportMax;
            PointType2d ViewportMin;
            PointType2d ViewportTickVector;
            DataPointToViewport(ScaleTransform, NormalizationTransform, DataRangeMin, ViewportMin);
            DataPointToViewport(ScaleTransform, NormalizationTransform, DataRangeMax, ViewportMax);
            DataVectorToViewport(ScaleTransform, NormalizationTransform, TickVector, ViewportTickVector);
            
            const float ViewportMidY = (ViewportMax.y + ViewportMin.y)/2.0f; 
            DrawLine(ViewportMin.x, ViewportMidY, ViewportMax.x, ViewportMidY);
            
            // try to fit one tick per 25 meters
            const int NumTicks = static_cast<int>((ViewportMax.x - ViewportMin.x) / ViewportTickVector.x);
            float X = ViewportMin.x;
            for (int nTick = 0; nTick < NumTicks; nTick++)
            {
                const float ThisTickHalfHeight = (nTick & 1) ? ViewportTickVector.x : ViewportTickVector.x/2.0f; 
                DrawLine(X, ViewportMidY - ThisTickHalfHeight, X, ViewportMidY + ThisTickHalfHeight);
                DrawText(std::format("{:}", nTick*25), {X, ViewportMidY + ThisTickHalfHeight});
                X += ViewportTickVector.x;
            }
        }
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
        FiringData.ZeroIn(0.01f, Environment);

        Ballistics::SolverParams Solver;
        Solver.MaxTime = 10.0f;
        Solver.TimeStep = 0.01f;
        Solver.MaxX = 300.0f;
        
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
    Solve();
    DrawTrajectory(true);
    DrawGrid();
    DrawText(std::format("Muzzle velocity is {:.1f}m/s", BulletData.MuzzleVelocityMs), {10.f, 25.f});
    DrawText(std::format("Zero distance is {:.1f}m", FiringData.ZeroDistance), {10.f, 40.f});
}
void AppUpdate()
{
    DrawGrid();
}
#else
int main(int /*argc*/, char** /*argv*/)
{
    Solve();
    return 0;
}
#endif
