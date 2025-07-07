
#include "Ballistics.h"
#include "Solver.h"
#include "Data.h"

#include <map>
#include <numbers>
#include <functional>
#include <array>

namespace Ballistics
{
    struct SolverBase
    {
        virtual ~SolverBase() = default;
        TrajectoryDataPoint Q;
        float DragFactor;
        EnvironmentData Environment;
        SolverParams Params;

        SolverBase(const FiringData& InFiringData, const EnvironmentData& Environment, const SolverParams& SolverParams)
            : Q(InFiringData),
            DragFactor(0.5f * Environment.AirDensity * InFiringData.Bullet.GetCrossSectionalArea() / InFiringData.Bullet.GetMassKg()),
            Environment(Environment),
            Params(SolverParams)
        {
        }
        virtual bool Completed() const
        {
            return Q.T >= Params.MaxTime || Q.DistanceY < 0.0f;
        }
        virtual bool Terminated() const
        {
            return Q.T >= Params.MaxTime;
        }
        virtual void Reset(const FiringData& InFiringData)
        {
            Q.Initialize(InFiringData);
        }
        virtual void Advance() = 0;
    };

    struct EulerSolver : SolverBase
    {
        EulerSolver(const FiringData& InFiringData, const EnvironmentData& InEnvironment, const SolverParams& SolverParams)
            : SolverBase(InFiringData, InEnvironment, SolverParams)
        {
        }

        virtual void Advance() override
        {
            const float DragAcceleration = -DragFactor * GetDragCoefficient(G7, Q.VelocityX, Environment.TKelvin) * (Q.VelocityX * Q.VelocityX);
            Q.VelocityX += DragAcceleration * Params.TimeStep;
            Q.VelocityY += Environment.Gravity * Params.TimeStep;
            Q.DistanceX += Q.VelocityX * Params.TimeStep;
            Q.DistanceY += Q.VelocityY * Params.TimeStep;
            Q.T += Params.TimeStep;
        }
    };

    struct HybridEulerRk4Solver : SolverBase
    {
        HybridEulerRk4Solver(const FiringData& InFiringData, const EnvironmentData& InEnvironment, const SolverParams& SolverParams)
            : SolverBase(InFiringData, InEnvironment, SolverParams)
        {
            LastQ.first = InFiringData.Bullet.MuzzleVelocityMs * cosf(InFiringData.ZeroAngle);
            LastQ.second = InFiringData.Bullet.MuzzleVelocityMs * sinf(InFiringData.ZeroAngle);
            
            VelocitySolver.Initialize(InFiringData.Bullet.MuzzleVelocityMs, SolverParams.TimeStep, [this](float V, float /* t */) -> float
                {
                    return -DragFactor * GetDragCoefficient(G7,V, this->Environment.TKelvin) * (V * V);
                });
        }

        virtual void Advance() override
        {
            const float FlightVelocity = VelocitySolver.Advance();
            const float AngleOfAttack = std::atan2f(LastQ.second, LastQ.first);
            
            Q.VelocityX = FlightVelocity * std::cosf(AngleOfAttack);
            Q.DistanceX += Q.VelocityX * Params.TimeStep;
            Q.VelocityY = FlightVelocity*std::sinf(AngleOfAttack) + Environment.Gravity * Params.TimeStep;
            Q.DistanceY += Q.VelocityY * Params.TimeStep;

            LastQ.first = Q.VelocityX;
            LastQ.second = Q.VelocityY;

            Q.T += Params.TimeStep;
        }

        void Reset(const FiringData& InFiringData) override
        {
            SolverBase::Reset(InFiringData);
            VelocitySolver.Reset();
            LastQ.first = InFiringData.Bullet.MuzzleVelocityMs * cosf(InFiringData.ZeroAngle);
            LastQ.second = InFiringData.Bullet.MuzzleVelocityMs * sinf(InFiringData.ZeroAngle);
        }

        Solver::RungeKutta4 VelocitySolver;
        std::pair<float, float> LastQ;
    };
    
	void SolveTrajectoryG7(std::vector<TrajectoryDataPoint>& OutElevation, const FiringData& InFiringData, const EnvironmentData& Environment, const SolverParams& InSolverParams)
	{
        HybridEulerRk4Solver Solver(InFiringData, Environment, InSolverParams);

        while (!Solver.Completed() && (InSolverParams.MaxX==0.0f || Solver.Q.DistanceX<InSolverParams.MaxX))
        {            
            Solver.Advance();
            OutElevation.emplace_back(Solver.Q);
        }
	}

    void FiringData::ZeroIn(float ToleranceM, const EnvironmentData& Environment)
    {
        if (ZeroDistance <= 0.0f)
        {
            return;
        }

	    SolverParams SolverParams;
        SolverParams.MaxTime = 10.0f;
        SolverParams.TimeStep = 0.01f;

        const float PrevHeight = Height;
        Height = ToleranceM;
        float MinAngle = 0.0f;
        float MaxAngle = static_cast<float>(std::numbers::pi) / 2.0f;
	    ZeroAngle = MinAngle + (MaxAngle - MinAngle) / 2.0f;
	    
	    HybridEulerRk4Solver Solver(*this, Environment, SolverParams);
        for (;;)
        {
            ZeroAngle = MinAngle + (MaxAngle - MinAngle) / 2.0f;
            Solver.Reset(*this);

            while (!Solver.Completed()
                &&
                Solver.Q.DistanceX < ZeroDistance-ToleranceM)
            {
                Solver.Advance();
            }

            // definite miss?
            if (fabsf(Solver.Q.DistanceY) > ToleranceM
                ||
                Solver.Q.DistanceX < ZeroDistance
                )
            {
                if (Solver.Q.DistanceY < 0.0f)
                {
                    // adjust up
                    MinAngle = ZeroAngle;
                }
                else
                {
                    // adjust down
                    MaxAngle = ZeroAngle;
                }
                continue;
            }

            // within target "box"?
            if (Solver.Q.DistanceX >= ZeroDistance - ToleranceM
                &&
                Solver.Q.DistanceX <= ZeroDistance + ToleranceM
                )
            {
                // done, reset to the original height
                Height = PrevHeight;
                break;
            }
            Height = PrevHeight;
            break;
        }
    }
}
