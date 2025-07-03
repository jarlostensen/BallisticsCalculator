
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

#if CALCULATE_ACCURATE_IMPACT_POINT
        // track the last DistanceY values for interpolation
        constexpr int CurveTaiLength = 4;
        std::array<std::pair<float,float>, CurveTaiLength> CurveTail;
        int CurveTailTail = 0;

#define CURVE_TAIL_INDEX() (CurveTailTail % CurveTaiLength)
#define CURVE_TAIL_POINT_AT(index) ((CurveTailTail+index) % CurveTaiLength)
#define NEXT_CURVE_TAIL_INDEX()\
CurveTailTail++
#endif

        // terminate if we hit the limit of being able to differentiate distances
        // if two solutions are within 1mm of each other we are not going to make meaningful progress
        for (;;)
        {
            ZeroAngle = MinAngle + (MaxAngle - MinAngle) / 2.0f;
            Solver.Reset(*this);
            
#if CALCULATE_ACCURATE_IMPACT_POINT
             const auto store_last_q = [&]
                 {
                    CurveTail[CURVE_TAIL_INDEX()].first = Solver.Q.DistanceX;
                    CurveTail[CURVE_TAIL_INDEX()].second = Solver.Q.DistanceY;
                    NEXT_CURVE_TAIL_INDEX();
                 };
#define UPDATE_CURVE() store_last_q()
#else
#define UPDATE_CURVE()
#endif

            while (!Solver.Completed()
                &&
                Solver.Q.DistanceX < ZeroDistance-ToleranceM)
            {
                UPDATE_CURVE()
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
                // done, reset to original height
                Height = PrevHeight;
                break;
            }
            
#if CALCULATE_ACCURATE_IMPACT_POINT
            // vertically close, but outside the box horisontally
            // interpolate height for better accuracy, add one more point to get the full set of 4
            store_last_q();
            Solver.Advance();
            store_last_q();

            Curves::CatmullRomSegment XSegment(
                CurveTail[CURVE_TAIL_POINT_AT(0)].first,
                CurveTail[CURVE_TAIL_POINT_AT(1)].first,
                CurveTail[CURVE_TAIL_POINT_AT(2)].first,
                CurveTail[CURVE_TAIL_POINT_AT(3)].first
            );
            Curves::CatmullRomSegment YSegment(
                CurveTail[CURVE_TAIL_POINT_AT(0)].second,
                CurveTail[CURVE_TAIL_POINT_AT(1)].second,
                CurveTail[CURVE_TAIL_POINT_AT(2)].second,
                CurveTail[CURVE_TAIL_POINT_AT(3)].second
            );

            float t = 0.0f;
            float dt = SolverParams.TimeStep;
            for (;;)
            {
                const float X = XSegment(t);
                if (X > ZeroDistance+ToleranceM)
                {
                    t -= dt;
                    dt /= 2.0f;
                }
                else if ( X > ZeroDistance-ToleranceM)
                {
                    break;
                }
                t += dt;
            }

            if (fabsf(YSegment(t)) < ToleranceM)
            {

            }
#endif
            Height = PrevHeight;
            break;
        }
    }
}
