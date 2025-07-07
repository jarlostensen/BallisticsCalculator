
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
            return Q.T >= Params.MaxTime || Q.Position.GetY() < 0.0f;
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
            const float DragAcceleration = -DragFactor * GetDragCoefficient(G7, Q.Velocity.GetX(), Environment.TKelvin) * (Q.Velocity.GetX() * Q.Velocity.GetX());
            Q.Velocity += {DragAcceleration*Params.TimeStep, Environment.Gravity*Params.TimeStep};
            Q.Position += Params.TimeStep * Q.Velocity;
            Q.T += Params.TimeStep;
        }
    };

    struct HybridEulerRk4Solver : SolverBase
    {
        HybridEulerRk4Solver(const FiringData& InFiringData, const EnvironmentData& InEnvironment, const SolverParams& SolverParams)
            : SolverBase(InFiringData, InEnvironment, SolverParams)
        {
            LastQ = { InFiringData.Bullet.MuzzleVelocityMs * cosf(InFiringData.ZeroAngle), InFiringData.Bullet.MuzzleVelocityMs * sinf(InFiringData.ZeroAngle) };
            
            VelocitySolver.Initialize(InFiringData.Bullet.MuzzleVelocityMs, SolverParams.TimeStep, [this](float V, float /* t */) -> float
                {
                    return -DragFactor * GetDragCoefficient(G7,V, this->Environment.TKelvin) * (V * V);
                });
        }

        virtual void Advance() override
        {
            const float FlightVelocity = VelocitySolver.Advance();
            const float AngleOfAttack = std::atan2f(LastQ.GetY(), LastQ.GetX());
            
            Q.Velocity = {FlightVelocity*std::cosf(AngleOfAttack), FlightVelocity*std::sinf(AngleOfAttack) + Environment.Gravity * Params.TimeStep};
            Q.Position += Params.TimeStep * Q.Velocity;
            LastQ = Q.Velocity;
            Q.T += Params.TimeStep;
        }

        void Reset(const FiringData& InFiringData) override
        {
            SolverBase::Reset(InFiringData);
            VelocitySolver.Reset();
            LastQ.SetX(InFiringData.Bullet.MuzzleVelocityMs * cosf(InFiringData.ZeroAngle));
            LastQ.SetY(InFiringData.Bullet.MuzzleVelocityMs * sinf(InFiringData.ZeroAngle));
        }

        Solver::RungeKutta4 VelocitySolver;
        Algebra::Vector2D LastQ;
    };
    
	void SolveTrajectoryG7(std::vector<TrajectoryDataPoint>& OutElevation, const FiringData& InFiringData, const EnvironmentData& Environment, const SolverParams& InSolverParams)
	{
        HybridEulerRk4Solver Solver(InFiringData, Environment, InSolverParams);

        while (!Solver.Completed() && (InSolverParams.MaxX==0.0f || Solver.Q.Position.GetX()<InSolverParams.MaxX))
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
                Solver.Q.Position.GetX() < ZeroDistance-ToleranceM)
            {
                Solver.Advance();
            }

            // definite miss?
            if (fabsf(Solver.Q.Position.GetY()) > ToleranceM
                ||
                Solver.Q.Position.GetX() < ZeroDistance
                )
            {
                if (Solver.Q.Position.GetY() < 0.0f)
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
            if (Solver.Q.Position.GetX() >= ZeroDistance - ToleranceM
                &&
                Solver.Q.Position.GetX() <= ZeroDistance + ToleranceM
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
