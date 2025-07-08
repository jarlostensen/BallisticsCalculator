#include <algorithm>
#include <format>
#include <iostream>
#include <iomanip>

#include "Application.h"

#include "Ballistics.h"

using namespace Plotter;
namespace
{
    Ballistics::BulletData BulletData;
    Ballistics::EnvironmentData Environment;
    Ballistics::FiringData FiringData;
    std::vector<Ballistics::TrajectoryDataPoint> TrajectoryDataPoints;
    Curve2D::PointInfo SelectedCurvePointInfo;
    bool bCurveSelected = false;
    PlotPtr TrajectoryPlot;

    void DrawUi()
    {
        if ( !TrajectoryPlot )
        {
            TrajectoryPlot = Plot::Create();
            Curve2D Curve;
            for (size_t nQ = 0; nQ < TrajectoryDataPoints.size(); ++nQ)
            {
                Curve.AddPoint(TrajectoryDataPoints[nQ].Position.GetX(), TrajectoryDataPoints[nQ].Position.GetY(), nQ);
            }
            Curve.SetColor(Magenta);
            TrajectoryPlot->AddCurve(std::move(Curve));
            Range2D PlotRange = TrajectoryPlot->GetExtents();

            const float Scale = 2.0f*static_cast<float>(std::numbers::pi) / TrajectoryDataPoints.size();
            const float HalfWidth = 0.5f * PlotRange.Width();
            const float HalfHeight = 0.5f * PlotRange.Height();
            for (size_t nQ = 0; nQ < (TrajectoryDataPoints.size()/2); ++nQ)
            {
                Curve.AddPoint(HalfWidth * (1.0f + cosf(Scale * nQ)), PlotRange.Min.GetY() + HalfHeight * (1.0f + sinf(Scale * nQ)));
            }
            Curve.SetColor(Red);
            TrajectoryPlot->AddCurve(std::move(Curve));
            PlotRange = TrajectoryPlot->GetExtents();

            const float CurveHeight = PlotRange.Height();
            const float CenterLineYPos = PlotRange.Min.GetY() + CurveHeight/2.0f;
            const float HeightLineXPos = PlotRange.Min.GetX() + 1.0f;
            TrajectoryPlot->AddLine({PlotRange.Min.GetX(), CenterLineYPos}, {PlotRange.Max.GetX(), CenterLineYPos}, Gray);
            TrajectoryPlot->AddLine({HeightLineXPos, PlotRange.Min.GetY()}, {HeightLineXPos, PlotRange.Max.GetY()}, Gray);
            TrajectoryPlot->AddLine({FiringData.ZeroDistance, PlotRange.Min.GetY()}, {FiringData.ZeroDistance, PlotRange.Max.GetY()}, Gray);
            TrajectoryPlot->AddLabel("Zero", {FiringData.ZeroDistance+0.1f,PlotRange.Max.GetY()}, Blue);

            // try to fit one tick per 25 meters
            Algebra::Vector2D TickVector = { 25.0f, PlotRange.Min.GetY() }; // every 25 meters
            
            int NumTicks = static_cast<int>((PlotRange.Max.GetX() - PlotRange.Min.GetX()) / TickVector.GetX());
            for (int nTick = 0; nTick < NumTicks; nTick++)
            {
                const float ThisTickHalfHeight = (nTick & 1) ? 0.02f : 0.01f;
                TrajectoryPlot->AddLine({TickVector.GetX(), CenterLineYPos - ThisTickHalfHeight}, {TickVector.GetX(), CenterLineYPos + ThisTickHalfHeight}, Gray);
                TrajectoryPlot->AddLabel(std::format("{:}", (nTick+1)*25), {TickVector.GetX(), CenterLineYPos - 1.1f * ThisTickHalfHeight});
                TickVector.SetX( TickVector.GetX() + 25.0f);
            }

            // vertical tick marks, one every 1/10th of the height
            NumTicks = 10;
            TickVector.SetX( HeightLineXPos );
            TickVector.SetY( PlotRange.Min.GetY() );
            const float TickHeight = PlotRange.Height() / static_cast<float>(NumTicks);
            for (int nTick = 0; nTick < NumTicks; nTick++)
            {
                TrajectoryPlot->AddLine({TickVector.GetX(), TickVector.GetY()}, {TickVector.GetX() + 1.0f, TickVector.GetY()},Gray);
                TrajectoryPlot->AddLabel(std::format("{:.1f}", TickVector.GetY()), {TickVector.GetX() + 1.1f, TickVector.GetY()});
                TickVector.SetY( TickVector.GetY() + TickHeight );
            }
        }
        
        DrawText(std::format("Muzzle velocity is {:.1f}m/s", BulletData.MuzzleVelocityMs), {10.f, 25.f});
        DrawText(std::format("Zero distance is {:.1f}m", FiringData.ZeroDistance), {10.f, 40.f});
        DrawText(std::format("Calibre {:.2f}mm, bullet weight {} grains", FiringData.Bullet.CallibreMm, static_cast<int>(FiringData.Bullet.MassGr)), { 200.0f, 25.0f });
        DrawText(std::format("Temperature {:.1f} Celcius", Ballistics::KelvinToCelcius(Environment.TKelvin)), {200.0f, 40.0f});
        
        if ( bCurveSelected )
        {
            Algebra::Vector2D Tangent = SelectedCurvePointInfo.Tangent;
            Algebra::Vector2D Normal = SelectedCurvePointInfo.Normal.Normalize();
            const float KineticEnergy = 0.5f * FiringData.Bullet.GetMassKg() * TrajectoryDataPoints[SelectedCurvePointInfo.MetaDataTag].Velocity.LengthSq();
            TrajectoryPlot->AddTransientLabel(std::format("x:{:.1f}m/s\ny:{:.1f}m/s\n{:.1f}J @ t:{:.001f}s",
                TrajectoryDataPoints[SelectedCurvePointInfo.MetaDataTag].Velocity.GetX(),
                TrajectoryDataPoints[SelectedCurvePointInfo.MetaDataTag].Velocity.GetY(),
                KineticEnergy,
                TrajectoryDataPoints[SelectedCurvePointInfo.MetaDataTag].T),
                TrajectoryDataPoints[SelectedCurvePointInfo.MetaDataTag].Position + 0.05f*Normal,
                DarkGray);
            TrajectoryPlot->AddTransientLine(TrajectoryDataPoints[SelectedCurvePointInfo.MetaDataTag].Position,
                TrajectoryDataPoints[SelectedCurvePointInfo.MetaDataTag].Position + Tangent, Blue);
        }

        Range2D ViewportExtents = GetRenderer()->GetViewportExtents();
        ViewportExtents.Min.SetX( ViewportExtents.Min.GetX() + ViewportExtents.Width()*0.1f);
        ViewportExtents.Min.SetY( ViewportExtents.Min.GetY() + ViewportExtents.Height()*0.15f);
        ViewportExtents.Max.SetX( ViewportExtents.Max.GetX() - ViewportExtents.Width()*0.1f);
        ViewportExtents.Max.SetY( ViewportExtents.Max.GetY() - ViewportExtents.Height()*0.15f);
        DrawPlot(TrajectoryPlot, ViewportExtents);
    }

    void Solve()
    {
        BulletData.MassGr = 155.0f;
        BulletData.G7BC = 0.275f;
        BulletData.MuzzleVelocityMs = 871.42f;
        BulletData.CallibreMm = Ballistics::Callibre308Mm;

        Environment.Gravity = -9.81f;
        Environment.TKelvin = 292.0f;
        Environment.AirPressure = 101325.0f;
        Environment.UpdateAirDensityFromTandP();
    
        FiringData.Bullet = BulletData;
        FiringData.Height = 1.0f;
        FiringData.ZeroDistance = 200.0f;
        // roughly one inch at 100m etc
        const float ToleranceM = (2.0f * (FiringData.ZeroDistance * 0.01f)) / 100.0f;

        FiringData.ZeroIn(ToleranceM, Environment);

        Ballistics::SolverParams Solver;
        Solver.MaxTime = 10.0f;
        Solver.TimeStep = 0.01f;
        Solver.MaxX = 300.0f;
        
        Ballistics::SolveTrajectoryG7(TrajectoryDataPoints, FiringData, Environment, Solver);
    }

}

#ifdef WITH_SDL
void AppInit()
{
    Solve();
}

void AppUpdate()
{
    DrawUi();
}

void AppHitDelegate(const Curve2D::PointInfo& PointInfo)
{
    SelectedCurvePointInfo = PointInfo;
    bCurveSelected = true;
}
#else
int main(int /*argc*/, char** /*argv*/)
{
    Solve();
    return 0;
}
#endif
