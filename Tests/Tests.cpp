
#include <Curves.h>
#include <Algebra.h>
#include <Ballistics.h>
#include <BulletData.h>
#include <Data.h>
#include <cassert>

namespace
{
    void TestBulletData()
    {
        Ballistics::BulletData BulletData;

        const std::string JsonData = R"({
            "bc_fn": "",
            "bc_g1": "0.29",
            "bc_g7": "",
            "company": "Hornady",
            "description": "Hornady .308 110gr V-MAX 23010",
            "diameter_in": "0.308",
            "name": "30 Cal .308 110 gr V-MAX®",
            "product_name": "V-MAX®",
            "sectional_density": "0.166",
            "sku": "23010",
            "type": "rifle",
            "weight_gr": "110"
        })";

        BulletData.ParseFromJsonString(JsonData);
    }
    
    void TestCatmullRom()
    {
        constexpr Curves::CatmullRomSegment1D Segment(-1.0f, 0.0f, 1.0f, 2.0f);

        float t = 0.0f;
        assert(Segment(0.0f) == 0.0f);
        assert(Segment(1.0f) == 1.0f);
        assert(Segment(0.5f) == 0.5f);
        static_assert(Segment.Curvature(0.0f) == 0.0f);
        static_assert(Segment.Curvature(1.0f) == 0.0f);
        
        std::vector<float> Samples;
        Segment.SampleAdaptively(Samples, 0.0f, 1.0f, 0.01f);
        assert(Samples.size() == 2);
        assert(Samples[0] == 0.0f);
        assert(Samples[1] == 1.0f);

        Samples.clear();
        Segment.SampleWithFwdDifference(Samples, 0.0f, 1.0f, 0.01f);
        assert(MathLib::NearlyEqual(Samples[0], 0.0f));
        //assert(MathLib::NearlyEqual(Samples[1], 1.0f));

        const Algebra::Vector2D P0{cosf(0.0f), sinf(0.0f)};
        const Algebra::Vector2D P1{ cosf(static_cast<float>(std::numbers::pi)/8.0f), sinf(static_cast<float>(std::numbers::pi)/8.0f)};
        const Algebra::Vector2D P2{ cosf(static_cast<float>(std::numbers::pi)*3.0f/8.0f), sinf(static_cast<float>(std::numbers::pi)*3.0f/8.0f)};
        const Algebra::Vector2D P3{ cosf(static_cast<float>(std::numbers::pi)/2.0f), sinf(static_cast<float>(std::numbers::pi)/2.0f) };
        Curves::CatmullRomSegment2D SinSegment(P0,P1,P2,P3);
        std::vector<Algebra::Vector2D> Samples2D;
        SinSegment.SampleAdaptively(Samples2D, 0.0f, 1.0f, 0.01f);
        assert(Samples2D.size() >= 4);
        assert(Samples2D[0].NearlyEqual(P1));
        assert(Samples2D[Samples2D.size()-1].NearlyEqual(P2));

        std::vector<Algebra::Vector2D> Samples2DFwdDiff;
        SinSegment.SampleWithFwdDifference(Samples2DFwdDiff, 0.0f, 1.0f, 0.01f);
        assert(Samples2DFwdDiff.size() >= 4);
        assert(Samples2DFwdDiff[0].NearlyEqual(P1));
        //assert(Samples2DFwdDiff[Samples2DFwdDiff.size() - 1].NearlyEqual(P2));

        for (t = 0.0f; t < 1.0f; t += 0.01f)
        {
            Algebra::Vector2D Normal = SinSegment.Normal(t).Normalize();
            Algebra::Vector2D Tangent = SinSegment.Tangent(t).Normalize();
            float DotProduct = Tangent.Dot(Normal);
            assert(MathLib::NearlyEqual(DotProduct, 0.0f));
        }
    }

    void TestZero()
    {
        Ballistics::BulletData BulletData;
        Ballistics::EnvironmentData Environment;
        Ballistics::FiringData FiringData;
        std::vector<Ballistics::TrajectoryDataPoint> TrajectoryDataPoints;

        BulletData.MassGr = 155.0f;
        BulletData.G7BC = 0.275f;
        BulletData.CallibreMm = Ballistics::Callibre308Mm;

        Environment.Gravity = -9.81f;
        Environment.TKelvin = 292.0f;
        Environment.AirPressure = 101325.0f;
        Environment.UpdateAirDensityFromTandP();

        FiringData.Bullet = BulletData;
        FiringData.Height = 10.0f;
        FiringData.ZeroDistance = 200.0f;
        FiringData.MuzzleVelocityMs = 871.42f;

        // roughly one inch at 100m etc
        const float ToleranceM = (2.0f * (FiringData.ZeroDistance * 0.01f)) / 100.0f;
        FiringData.ZeroIn(Ballistics::G7, ToleranceM, Environment);
        assert(FiringData.ZeroAngle > 0.0f);
        FiringData.ZeroIn(Ballistics::G1, ToleranceM, Environment);
        assert(FiringData.ZeroAngle > 0.0f);
    }

    void TestAlgebra()
    {
        constexpr Algebra::Matrix2D UnitMatrix;
        static_assert(UnitMatrix.Determinant() == 1.0f);

        constexpr Algebra::Vector2D UnitVector(1.0f, 0.0f);
        static_assert(UnitVector.LengthSq()==1.0f);
        
        Algebra::Matrix2D M1 = Algebra::Matrix2D::Rotation(static_cast<float>(std::numbers::pi) / 4.0f);
        std::optional<Algebra::Matrix2D> M1Inv = M1.Inverse();
        assert(M1Inv.has_value());
        Algebra::Matrix2D M2 = M1 * M1Inv.value();
        assert(M2.Determinant() == 1.0f);
        constexpr Algebra::Vector2D UnitVectorX(1.0f, 0.0f);
        constexpr Algebra::Vector2D UnitVectorY(1.0f, 0.0f);
        const Algebra::Vector2D RotatedUnitVectorX = M1 * UnitVectorX;
        assert(MathLib::NearlyEqual(RotatedUnitVectorX.LengthSq(), 1.0f));
        assert(MathLib::NearlyEqual(RotatedUnitVectorX.GetX(), RotatedUnitVectorX.GetY()));
        const Algebra::Vector2D RotatedUnitVectorY = M1 * UnitVectorY;
        assert(MathLib::NearlyEqual(RotatedUnitVectorY.LengthSq(), 1.0f));
        assert(MathLib::NearlyEqual(RotatedUnitVectorY.GetX(), RotatedUnitVectorY.GetY()));
    }
}

int main(int argc, char* argv[])
{
    TestBulletData();
    TestCatmullRom();
    TestZero();
    TestAlgebra();
    return 0;
}
