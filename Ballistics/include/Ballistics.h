#pragma once
#include <vector>
#include <Algebra.h>
#include "BulletData.h"
#include "Data.h"

namespace Ballistics
{
	constexpr float Callibre308Mm = 7.62f;
	constexpr float MsToFtS = 3.2804f;

	
	/**
	 * @brief Represents the environmental parameters affecting ballistic calculations.
	 *
	 * This structure contains essential environmental properties such as temperature, gravity,
	 * air density, and air pressure. It also provides functionality to update the air density
	 * based on the current temperature and pressure values using the ideal gas law approximation.
	 */
	struct EnvironmentData
	{
		float	TKelvin;
		float	Gravity;
		float	AirDensity;
		float	AirPressure;

		constexpr void UpdateAirDensityFromTandP()
		{
			AirDensity = AirPressure / (287.05f * TKelvin);
		}
	};

	constexpr float KelvinToCelcius(float TK)
	{
		return TK - 272.15f;
	}

	/**
	 * @brief Represents the configuration and parameters required for firing calculations.
	 *
	 * This structure combines critical data for ballistic computations, including information
	 * about the bullet, the zeroing parameters, and the height of the shooter or firing platform.
	 * It provides functionality to compute the firing angle required to achieve a specified zero distance.
	 */
	struct FiringData
	{
		BulletData Bullet;
		float	MuzzleVelocityMs = 0.0f;
		float	ZeroDistance = 0.0f;
		float	ZeroAngle = 0.0f;
		float	Height = 0.0f;

		void ZeroIn(const DragTableType& InDragTable, float ToleranceMm, const EnvironmentData& Environment);
	};

	struct SolverParams
	{
		float TimeStep = 0.0f;
		float MaxTime = 0.0f;
		float MaxX = 0.0f;
	};

	/**
	 * @brief Represents a single point in the trajectory of a projectile.
	 *
	 * This structure captures the state of a projectile at a given time during its flight.
	 * It includes information such as the current velocity components, the distances traveled
	 * in both horizontal and vertical directions, and the elapsed time.
	 * The structure also provides functionality to initialize its state based on input firing data.
	 */
	struct TrajectoryDataPoint
	{
		Algebra::Vector2D Velocity;
		Algebra::Vector2D Position;
		float T;

		constexpr TrajectoryDataPoint() = default;
		explicit TrajectoryDataPoint(FiringData InFiringData)
		{
			Initialize(InFiringData);
		}

		constexpr TrajectoryDataPoint& Initialize(FiringData InFiringData)
		{
			T = 0.0f;
			Position.SetX(0.0f);
			Position.SetY(InFiringData.Height);
			Velocity.SetX(InFiringData.MuzzleVelocityMs * cosf(InFiringData.ZeroAngle));
			Velocity.SetY(InFiringData.MuzzleVelocityMs * sinf(InFiringData.ZeroAngle));

			return *this;
		}
	};

	/**
	 * Calculate trajectory of projectile using G7 tabular data
	 * @param OutTrajectoryDataPoints 
	 * @param InFiringData 
	 * @param Environment 
	 * @param Solver 
	 */
	void SolveTrajectory(const DragTableType& InDragTable, std::vector<TrajectoryDataPoint>& OutTrajectoryDataPoints, const FiringData & InFiringData, const EnvironmentData & Environment, const SolverParams & Solver);
}