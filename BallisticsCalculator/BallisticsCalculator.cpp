
#include "BallisticsCalculator.h"
#include "Curves.h"
#include "Solver.h"

#include <map>
#include <numbers>
#include <functional>
#include <array>

namespace Ballistics
{
	using DragTableType = std::map<float, float>;
	const DragTableType G7 =
	{
        {0.00f, 0.1198f},
        {0.05f, 0.1197f},
        {0.10f, 0.1196f},
        {0.15f, 0.1194f},
        {0.20f, 0.1193f},
        {0.25f, 0.1194f},
        {0.30f, 0.1194f},
        {0.35f, 0.1194f},
        {0.40f, 0.1193f},
        {0.45f, 0.1193f},
        {0.50f, 0.1194f},
        {0.55f, 0.1193f},
        {0.60f, 0.1194f},
        {0.65f, 0.1197f},
        {0.70f, 0.1202f},
        {0.725f, 0.1207f},
        {0.75f, 0.1215f},
        {0.775f, 0.1226f},
        {0.80f, 0.1242f},
        {0.825f, 0.1266f},
        {0.85f, 0.1306f},
        {0.875f, 0.1368f},
        {0.90f, 0.1464f},
        {0.925f, 0.1660f},
        {0.95f, 0.2054f},
        {0.975f, 0.2993f},
        {1.0f, 0.3803f},
        {1.025f, 0.4015f},
        {1.05f, 0.4043f},
        {1.075f, 0.4034f},
        {1.10f, 0.4014f},
        {1.125f, 0.3987f},
        {1.15f, 0.3955f},
        {1.20f, 0.3884f},
        {1.25f, 0.3810f},
        {1.30f, 0.3732f},
        {1.35f, 0.3657f},
        {1.40f, 0.3580f},
        {1.50f, 0.3440f},
        {1.55f, 0.3376f},
        {1.60f, 0.3315f},
        {1.65f, 0.3260f},
        {1.70f, 0.3209f},
        {1.75f, 0.3160f},
        {1.80f, 0.3117f},
        {1.85f, 0.3078f},
        {1.90f, 0.3042f},
        {1.95f, 0.3010f},
        {2.00f, 0.2980f},
        {2.05f, 0.2951f},
        {2.10f, 0.2922f},
        {2.15f, 0.2892f},
        {2.20f, 0.2864f},
        {2.25f, 0.2835f},
        {2.30f, 0.2807f},
        {2.35f, 0.2779f},
        {2.40f, 0.2752f},
        {2.45f, 0.2725f},
        {2.50f, 0.2697f},
        {2.55f, 0.2670f},
        {2.60f, 0.2643f},
        {2.65f, 0.2615f},
        {2.70f, 0.2588f},
        {2.75f, 0.2561f},
        {2.80f, 0.2533f},
        {2.85f, 0.2506f},
        {2.90f, 0.2479f},
        {2.95f, 0.2451f},
        {3.00f, 0.2424f},
        {3.10f, 0.2368f},
        {3.20f, 0.2313f},
        {3.30f, 0.2258f},
        {3.40f, 0.2205f},
        {3.50f, 0.2154f},
        {3.60f, 0.2106f},
        {3.70f, 0.2060f},
        {3.80f, 0.2017f},
        {3.90f, 0.1975f},
        {4.00f, 0.1935f},
        {4.20f, 0.1861f},
        {4.40f, 0.1793f},
        {4.60f, 0.1730f},
        {4.80f, 0.1672f},
        {5.00f, 0.1618f},
	};

    float SpeedToMach(float SpeedMs, float TemperatureK)
    {
        constexpr float Gamma = 1.4f;
        constexpr float R = 287.05f;
        return SpeedMs / sqrtf(Gamma * R * TemperatureK);
    }

    float GetNearestDragCoefficient(float Speed, float TemperatureK)
    {
        Speed = SpeedToMach(Speed, TemperatureK);

        // do a simple linear interpolation between points in the table
        std::pair<float, float> PrevPoint{ 0.0f,0.0f };
        for (const auto& DragTableEntry : G7)
        {
            if (DragTableEntry.first >= Speed)
            {
                if (PrevPoint.first != 0.0f)
                {
                    const float Scale = (Speed - PrevPoint.first)/(DragTableEntry.first - PrevPoint.first);
                    return PrevPoint.second + Scale * (DragTableEntry.second - PrevPoint.second);
                }
                return DragTableEntry.second;
            }
            PrevPoint = DragTableEntry;
        }
        return 0.0f;
    }

    struct SolverState
    {
        TrajectoryDataPoint Q;
        const float DragFactor;
        EnvironmentData Environment;
        SolverParams Params;

        SolverState(const FiringData& InFiringData, const EnvironmentData& Environment, const SolverParams& SolverParams)
            : Q(InFiringData),
            DragFactor(0.5f * Environment.AirDensity * InFiringData.Bullet.GetCrossSectionalArea() / InFiringData.Bullet.GetMassKg()),
            Environment(Environment),
            Params(SolverParams)
        {
        }

    };

    struct SolverBase
    {
        TrajectoryDataPoint Q;
        const float DragFactor;
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
        EulerSolver(const FiringData& InFiringData, const EnvironmentData& Environment, const SolverParams& SolverParams)
            : SolverBase(InFiringData, Environment, SolverParams)
        {
        }

        virtual void Advance() override
        {
            const float DragAcceleration = -DragFactor * GetNearestDragCoefficient(Q.VelocityX, Environment.TKelvin) * (Q.VelocityX * Q.VelocityX);
            Q.VelocityX += DragAcceleration * Params.TimeStep;
            Q.VelocityY += Environment.Gravity * Params.TimeStep;
            Q.DistanceX += Q.VelocityX * Params.TimeStep;
            Q.DistanceY += Q.VelocityY * Params.TimeStep;
            Q.T += Params.TimeStep;
        }
    };

    struct HybridEulerRk4Solver : SolverBase
    {
        HybridEulerRk4Solver(const FiringData& InFiringData, const EnvironmentData& Environment, const SolverParams& SolverParams)
            : SolverBase(InFiringData, Environment, SolverParams)
        {
            VxSolver.Initialize(Q.VelocityX, SolverParams.TimeStep, [this](float Vx, float /* t */) -> float
                {
                    return -DragFactor * GetNearestDragCoefficient(Vx, this->Environment.TKelvin) * (Vx * Vx);
                });
        }

        virtual void Advance() override
        {
            Q.VelocityX = VxSolver.Advance();
            Q.DistanceX += Q.VelocityX * Params.TimeStep;

            // Vy
            Q.VelocityY += Environment.Gravity * Params.TimeStep;
            Q.DistanceY += Q.VelocityY * Params.TimeStep;

            Q.T += Params.TimeStep;
        }

        virtual void Reset(const FiringData& InFiringData)
        {
            SolverBase::Reset(InFiringData);
            VxSolver.Reset();
        }

        Solver::RungeKutta4 VxSolver;
    };

    float SolveImpactAtHorisontalDistance(float HoristontalDistance, const FiringData& InFiringData, const EnvironmentData& Environment, const SolverParams& InSolverParams)
    {
        HybridEulerRk4Solver Solver(InFiringData, Environment, InSolverParams);

        while (!Solver.Completed() && Solver.Q.DistanceX < HoristontalDistance)
        {
            Solver.Advance();
        }
        return Solver.Q.DistanceY;
    }

	void SolveTrajectoryG7(std::vector<TrajectoryDataPoint>& OutElevation, const FiringData& InFiringData, const EnvironmentData& Environment, const SolverParams& InSolverParams)
	{
        HybridEulerRk4Solver Solver(InFiringData, Environment, InSolverParams);

        while (!Solver.Completed())
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

        HybridEulerRk4Solver Solver(*this, Environment, SolverParams);

        float MinAngle = 0.0f;
        float MaxAngle = (float)std::numbers::pi / 2.0f;

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

        // terminate if we hit the limit of being able to differentiate distances
        float SolutionDelta = std::numeric_limits<float>::max();
        // if two solutions are within 1mm of each other we are not going to make meaningful progress
        while(SolutionDelta >= 0.001f)
        {
            ZeroAngle = MinAngle + (MaxAngle - MinAngle) / 2.0f;
            Solver.Reset(*this);
            
            float PrevDistanceX = 0.0f;
            const auto store_last_q = [&]()
                {
                    CurveTail[CurveTailStartIndex] = Solver.Q.DistanceY;
                    curve_tail_index(+1);
                };

            while (!Solver.Completed() && Solver.Q.DistanceX < ZeroDistance)
            {
                store_last_q();
                PrevDistanceX = Solver.Q.DistanceX;
                Solver.Advance();
            }

            if (Solver.Q.DistanceX >= ZeroDistance)
            {
                float ImpactHeight;
                if (Solver.Q.DistanceX > ZeroDistance)
                {
                    // interpolate height for better accuracy, add one more point to get the full set of 4
                    store_last_q();
                    Curves::CatmullRomSegment CatmullRomSegment(
                        CurveTail[curve_tail_index(0)],
                        CurveTail[curve_tail_index(1)],
                        CurveTail[curve_tail_index(2)],
                        CurveTail[curve_tail_index(3)]
                    );
                    // estimate curve parameter by fraction of X distance
                    const float t = (ZeroDistance - PrevDistanceX) / (Solver.Q.DistanceX - PrevDistanceX);
                    ImpactHeight = CatmullRomSegment(t);
                }
                else
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
            
            SolutionDelta = fabsf(Solver.Q.DistanceY - CurveTail[curve_tail_index(-1)]);
        }
    }
}
