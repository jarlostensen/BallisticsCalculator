﻿#include "Data.h"

/* Tables ported to C++ from https://github.com/dbookstaber/py_ballistics/blob/master/py_ballisticcalc/drag_tables.py */

namespace Ballistics
{
	const DragTableType G1 =
	{
	{0.00f, 0.2629f},
    {0.05f, 0.2558f},
    {0.10f, 0.2487f},
    {0.15f, 0.2413f},
    {0.20f, 0.2344f},
    {0.25f, 0.2278f},
    {0.30f, 0.2214f},
    {0.35f, 0.2155f},
    {0.40f, 0.2104f},
    {0.45f, 0.2061f},
    {0.50f, 0.2032f},
    {0.55f, 0.2020f},
    {0.60f, 0.2034f},
    {0.70f, 0.2165f},
    {0.725f, 0.2230f},
    {0.75f, 0.2313f},
    {0.775f, 0.2417f},
    {0.80f, 0.2546f},
    {0.825f, 0.2706f},
    {0.85f, 0.2901f},
    {0.875f, 0.3136f},
    {0.90f, 0.3415f},
    {0.925f, 0.3734f},
    {0.95f, 0.4084f},
    {0.975f, 0.4448f},
    {1.0f, 0.4805f},
    {1.025f, 0.5136f},
    {1.05f, 0.5427f},
    {1.075f, 0.5677f},
    {1.10f, 0.5883f},
    {1.125f, 0.6053f},
    {1.15f, 0.6191f},
    {1.20f, 0.6393f},
    {1.25f, 0.6518f},
    {1.30f, 0.6589f},
    {1.35f, 0.6621f},
    {1.40f, 0.6625f},
    {1.45f, 0.6607f},
    {1.50f, 0.6573f},
    {1.55f, 0.6528f},
    {1.60f, 0.6474f},
    {1.65f, 0.6413f},
    {1.70f, 0.6347f},
    {1.75f, 0.6280f},
    {1.80f, 0.6210f},
    {1.85f, 0.6141f},
    {1.90f, 0.6072f},
    {1.95f, 0.6003f},
    {2.00f, 0.5934f},
    {2.05f, 0.5867f},
    {2.10f, 0.5804f},
    {2.15f, 0.5743f},
    {2.20f, 0.5685f},
    {2.25f, 0.5630f},
    {2.30f, 0.5577f},
    {2.35f, 0.5527f},
    {2.40f, 0.5481f},
    {2.45f, 0.5438f},
    {2.50f, 0.5397f},
    {2.60f, 0.5325f},
    {2.70f, 0.5264f},
    {2.80f, 0.5211f},
    {2.90f, 0.5168f},
    {3.00f, 0.5133f},
    {3.10f, 0.5105f},
    {3.20f, 0.5084f},
    {3.30f, 0.5067f},
    {3.40f, 0.5054f},
    {3.50f, 0.5040f},
    {3.60f, 0.5030f},
    {3.70f, 0.5022f},
    {3.80f, 0.5016f},
    {3.90f, 0.5010f},
    {4.00f, 0.5006f},
    {4.20f, 0.4998f},
    {4.40f, 0.4995f},
    {4.60f, 0.4992f},
    {4.80f, 0.4990f},
    {5.00f, 0.4988f}
	};
	
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

    namespace 
    {
    	float SpeedToMach(float SpeedMs, float TemperatureK)
    	{
    		constexpr float Gamma = 1.4f;
    		constexpr float R = 287.05f;
    		return SpeedMs / sqrtf(Gamma * R * TemperatureK);
    	}
    }
	
	float GetDragCoefficient(const DragTableType& Table, float Speed, float TemperatureK)
	{
		Speed = SpeedToMach(Speed, TemperatureK);
		//NOTE: returns first element >= Speed ("not less than")
		DragTableType::const_iterator DragTableIter = Table.lower_bound(Speed);
		if (DragTableIter!=Table.end())
		{
			const auto UpperEntry = *DragTableIter;
			std::pair<float, float> LowerEntry{0.0f,0.0f};
			if ( DragTableIter!=Table.begin() )
			{
				LowerEntry = *(--DragTableIter);
			}
			const float Scale = (Speed - LowerEntry.first) / (UpperEntry.first - LowerEntry.first);
			return LowerEntry.second + Scale*(UpperEntry.second - LowerEntry.second);
		}
		return 0.0f;
	}
}