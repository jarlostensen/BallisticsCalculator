#pragma once
#include <vector>
#include <numbers>
#include <Algebra.h>

namespace Ballistics
{
	constexpr float Callibre308Mm = 7.62f;
	constexpr float MsToFtS = 3.2804f;

	/**
	 * @brief Represents the data related to a bullet used in ballistic calculations.
	 *
	 * This structure encapsulates essential properties of a bullet, such as its mass,
	 * muzzle velocity, ballistic coefficient, and caliber.
	 */
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

		void UpdateAirDensityFromTandP()
		{
			AirDensity = AirPressure / (287.05f * TKelvin);
		}
	};

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

		//float VelocityX;
		//float VelocityY;
		//float DistanceY;
		//float DistanceX;
		float T;

		TrajectoryDataPoint() = default;
		explicit TrajectoryDataPoint(FiringData InFiringData)
		{
			Initialize(InFiringData);
		}

		TrajectoryDataPoint& Initialize(FiringData InFiringData)
		{
			T = 0.0f;
			/*DistanceX = 0.0f;
			DistanceY = InFiringData.Height;
			VelocityX = InFiringData.Bullet.MuzzleVelocityMs * cosf(InFiringData.ZeroAngle);
			VelocityY = InFiringData.Bullet.MuzzleVelocityMs * sinf(InFiringData.ZeroAngle);*/

			Position.SetX(0.0f);
			Position.SetY(InFiringData.Height);
			Velocity.SetX(InFiringData.Bullet.MuzzleVelocityMs * cosf(InFiringData.ZeroAngle));
			Velocity.SetY(InFiringData.Bullet.MuzzleVelocityMs * sinf(InFiringData.ZeroAngle));

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
	void SolveTrajectoryG7(std::vector<TrajectoryDataPoint>& OutTrajectoryDataPoints, const FiringData & InFiringData, const EnvironmentData & Environment, const SolverParams & Solver);
}