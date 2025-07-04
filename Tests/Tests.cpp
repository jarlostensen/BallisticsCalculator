
#include <Curves.h>
#include <Ballistics.h>
#include <cassert>

namespace
{
    void TestCatmullRom()
    {
        Curves::CatmullRomSegment1D Segment(-1.0f, 0.0f, 1.0f, 2.0f);

        float t = 0.0f;
        assert(Segment(0.0f) == 0.0f);
        assert(Segment(1.0f) == 1.0f);
        assert(Segment(0.5f) == 0.5f);

        std::vector<float> Samples;
        Segment.SampleAdaptively(Samples, 0.0f, 1.0f, 0.01f);
        assert(Samples.size() == 2);
        assert(Samples[0] == 0.0f);
        assert(Samples[1] == 1.0f);

        const Algebra::Vector2D P0 = {cosf(0.0f), sinf(0.0f)};
        const Algebra::Vector2D P1 = { cosf(static_cast<float>(std::numbers::pi)/8.0f), sinf(static_cast<float>(std::numbers::pi)/8.0f)};
        const Algebra::Vector2D P2 = { cosf(static_cast<float>(std::numbers::pi)*3.0f/8.0f), sinf(static_cast<float>(std::numbers::pi)*3.0f/8.0f)};
        const Algebra::Vector2D P3 = { cosf(static_cast<float>(std::numbers::pi)/2.0f), sinf(static_cast<float>(std::numbers::pi)/2.0f) };
        Curves::CatmullRomSegment2D SinSegment(P0,P1,P2,P3);
        std::vector<Algebra::Vector2D> Samples2D;
        SinSegment.SampleAdaptively(Samples2D, 0.0f, 1.0f, 0.01f);
        assert(Samples2D.size() >= 4);
        assert(fabsf(Samples2D[0] - P1) < 0.001f);
        assert(fabsf(Samples2D[Samples2D.size()-1] - P2) < 0.001f);
    }

    void TestZero()
    {
        Ballistics::BulletData BulletData;
        Ballistics::EnvironmentData Environment;
        Ballistics::FiringData FiringData;
        std::vector<Ballistics::TrajectoryDataPoint> TrajectoryDataPoints;

        BulletData.MassGr = 155.0f;
        BulletData.G7BC = 0.275f;
        BulletData.MuzzleVelocityMs = 871.42f;
        BulletData.CallibreMm = Ballistics::Callibre308Mm;

        Environment.Gravity = -9.81f;
        Environment.TKelvin = 292.0f;
        Environment.AirPressure = 101325.0f;
        Environment.UpdateAirDensityFromTandP();

        FiringData.Bullet = BulletData;
        FiringData.Height = 10.0f;
        FiringData.ZeroDistance = 200.0f;

        // roughly one inch at 100m etc
        const float ToleranceM = (2.0f * (FiringData.ZeroDistance * 0.01f)) / 100.0f;

        FiringData.ZeroIn(ToleranceM, Environment);

        assert(FiringData.ZeroAngle > 0.0f);
    }
}

int main(int argc, char* argv[])
{
    TestCatmullRom();
    TestZero();
    return 0;
}
