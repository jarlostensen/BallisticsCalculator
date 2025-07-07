#pragma once
#include <cmath>

namespace MathLib
{
	constexpr float Epsilon = 1e-6f;
	inline bool NearlyEqual(float Lhs, float Rhs)
	{
		return std::fabsf(Lhs - Rhs) <= Epsilon;
	}
}
