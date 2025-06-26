
#include "BallisticsCalculator.h"
#include "Curves.h"
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
        Height = 0.0f;
        float MinAngle = 0.0f;
        float MaxAngle = static_cast<float>(std::numbers::pi) / 2.0f;
	    ZeroAngle = MinAngle + (MaxAngle - MinAngle) / 2.0f;
	    
	    HybridEulerRk4Solver Solver(*this, Environment, SolverParams);

        // track the last DistanceY values for interpolation
        constexpr int CurveTaiLength = 4;
        std::array<float, CurveTaiLength> CurveTail;
        CurveTail.fill(std::numeric_limits<float>::max());
        int CurveTailStartIndex = 0;

        const auto curve_tail_index = [&](int offset)
            {
                CurveTailStartIndex = (CurveTailStartIndex + offset) % CurveTaiLength;
                return (CurveTailStartIndex = (CurveTailStartIndex < 0) ? CurveTailStartIndex + CurveTaiLength : CurveTailStartIndex);
            };
        
        float SolutionDelta = std::numeric_limits<float>::max();
	    float PrevImpactHeight = 0.0f;
        // terminate if we hit the limit of being able to differentiate distances
        // if two solutions are within 1mm of each other we are not going to make meaningful progress
        while(SolutionDelta > 0.001f)
        {
            ZeroAngle = MinAngle + (MaxAngle - MinAngle) / 2.0f;
            Solver.Reset(*this);
            
            //float PrevDistanceX = 0.0f;
            // const auto store_last_q = [&]()
            //     {
            //         CurveTail[CurveTailStartIndex] = Solver.Q.DistanceY;
            //         curve_tail_index(+1);
            //     };

            while (!Solver.Completed() && Solver.Q.DistanceX < ZeroDistance)
            {
                // store_last_q();
                // PrevDistanceX = Solver.Q.DistanceX;
                Solver.Advance();
            }

            float ImpactHeight = 0.0f;
            if (Solver.Q.DistanceX >= ZeroDistance)
            {
                // if (Solver.Q.DistanceX > ZeroDistance)
                // {
                //     // interpolate height for better accuracy, add one more point to get the full set of 4
                //     store_last_q();
                //     Curves::CatmullRomSegment CatmullRomSegment(
                //         CurveTail[curve_tail_index(0)],
                //         CurveTail[curve_tail_index(1)],
                //         CurveTail[curve_tail_index(2)],
                //         CurveTail[curve_tail_index(3)]
                //     );
                //     // estimate curve parameter by fraction of X distance
                //     const float t = (ZeroDistance - PrevDistanceX) / (Solver.Q.DistanceX - PrevDistanceX);
                //     ImpactHeight = CatmullRomSegment(t);
                // }
                // else
                {
                    ImpactHeight = Solver.Q.DistanceY;
                }
                if(fabsf(ImpactHeight) <= ToleranceM)
                {
                    Height = PrevHeight;
                    return;
                }
            }

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

            SolutionDelta = fabsf(ImpactHeight - PrevImpactHeight);
            PrevImpactHeight = ImpactHeight;
        }
    }
}
