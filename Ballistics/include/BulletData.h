#pragma once
#include <numbers>
#include <string>

namespace Ballistics
{
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
        float   G1BC = 0.0f;
        float	G7BC = 0.0f;
        float	CallibreMm = 0.0f;
        std::string Name;
        std::string Description;
        std::string Company;
		
        constexpr float GetMassKg() const
        {
            return MassGr * 0.00006479891f;
        }

        float GetCrossSectionalArea() const
        {
            return static_cast<float>(std::numbers::pi) * 0.25f * (CallibreMm * CallibreMm / 1000000.0f);
        }

        /**
         * Parse from bullet data as found in https://github.com/ammolytics/projectiles/blob/develop/data/lapua.json 
         *
         * e.g.
        *  {
                "bc_fn": "",
                "bc_g1": "0.029",
                "bc_g7": "",
                "company": "Lapua",
                "description": "Lapua .314 83gr Wadcutter 4HL8023",
                "diameter_in": "0.314",
                "product_name": "Wadcutter",
                "weight_gr": "83"
           }
         * 
         * @param JsonString 
         * @return 
         */
        bool ParseFromJsonString(const std::string& JsonString);
    };

}
