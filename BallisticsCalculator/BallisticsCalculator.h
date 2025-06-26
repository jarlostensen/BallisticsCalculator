#pragma once
#include <vector>
#include <numbers>

namespace Ballistics
{
	constexpr float Callibre308Mm = 7.62f;
	constexpr float MsToFtS = 3.2804f;

	struct BulletData
	{
		float	MassGr = 0.0f;
		float	MuzzleVelocityMs = 0.0f;
		float	G7BC = 0.0f;
		float	CallibreMm = 0.0f;
		
		float GetMassKg() const
		{
			return MassGr * 0.00006479891f;
		}

		float GetCrossSectionalArea() const
		{
			return (float)std::numbers::pi * 0.25f * (CallibreMm * CallibreMm / 1000000.0f);
		}
	};

	struct EnvironmentData
	{
		float	TKelvin;
		float	Gravity;
		float	AirDensity;
		float	AirPressure;

		void UpdateAirDensityFromTandP()
		{
			AirDensity = AirPressure / (287.05f * TKelvin);
		}
	};

	struct FiringData
	{
		BulletData Bullet;
		float	ZeroDistance = 0.0f;
		float	ZeroAngle = 0.0f;
		float	Height = 0.0f;

		void ZeroIn(float ToleranceMm, const EnvironmentData& Environment);
	};

	struct SolverParams
	{
		float TimeStep = 0.0f;
		float MaxTime = 0.0f;
		float MaxX = 0.0f;
	};

	struct TrajectoryDataPoint
	{
		float VelocityX;
		float VelocityY;
		float DistanceY;
		float DistanceX;
		float T;

		TrajectoryDataPoint() = default;
		explicit TrajectoryDataPoint(FiringData InFiringData)
		{
			Initialize(InFiringData);
		}

		TrajectoryDataPoint& Initialize(FiringData InFiringData)
		{
			T = 0.0f;
			DistanceX = 0.0f;
			DistanceY = InFiringData.Height;
			VelocityX = InFiringData.Bullet.MuzzleVelocityMs * cosf(InFiringData.ZeroAngle);
			VelocityY = InFiringData.Bullet.MuzzleVelocityMs * sinf(InFiringData.ZeroAngle);
			return *this;
		}
	};

	void SolveTrajectoryG7(std::vector<TrajectoryDataPoint>& OutTrajectoryDataPoints, const FiringData & InFiringData, const EnvironmentData & Environment, const SolverParams & Solver);
}