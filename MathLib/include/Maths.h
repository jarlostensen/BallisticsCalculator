#pragma once
#include <cmath>

namespace MathLib
{
	inline bool NearlyEqual(float Lhs, float Rhs)
	{
		return std::fabsf(Lhs - Rhs) <= 1e-6f;
	}
}
