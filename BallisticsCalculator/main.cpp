#include <iostream>
#include <iomanip>

#include "BallisticsCalculator.h"
#include "Curves.h"

int main()
{
    Ballistics::BulletData BulletData;
    BulletData.MassGr = 155.0f;
    BulletData.G7BC = 0.275f;
    BulletData.MuzzleVelocityMs = 871.42f;
    BulletData.CallibreMm = Ballistics::Callibre308Mm;

    Ballistics::EnvironmentData Environment;
    Environment.Gravity = -9.81f;
    Environment.TKelvin = 292.0f;
    Environment.AirPressure = 101325.0f;
    Environment.UpdateAirDensityFromTandP();
    
    Ballistics::FiringData FiringData;
    FiringData.Bullet = BulletData;
    FiringData.Height = 10.0f;
    FiringData.ZeroDistance = 100.0f;
    FiringData.ZeroIn(0.01f, Environment);

    Ballistics::SolverParams Solver;
    Solver.MaxTime = 10.0f;
    Solver.TimeStep = 0.01f;

    std::vector<Ballistics::TrajectoryDataPoint> TrajectoryDataPoints;
    Ballistics::SolveTrajectoryG7(TrajectoryDataPoints, FiringData, Environment, Solver);

    std::cout << "T\tX\tY\tVx\n";
    for (const Ballistics::TrajectoryDataPoint& Q : TrajectoryDataPoints)
    {
        const float KineticEnergy = 0.5f * BulletData.GetMassKg() * (Q.VelocityX * Q.VelocityX);
        std::cout << std::fixed << std::setprecision(2) << Q.T << "\t" << Q.DistanceX << "\t" << (Q.DistanceY-FiringData.Height)*100.0f << " cm\t" << Q.VelocityX << " m/s (" << (Q.VelocityX * Ballistics::MsToFtS) << " ft/s)\t" << KineticEnergy << " Joules\n";
    }
}
