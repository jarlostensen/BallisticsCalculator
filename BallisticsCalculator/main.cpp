#include <algorithm>
#include <format>
#include <iostream>
#include <iomanip>

#include "Application.h"
#include "Plotter.h"

#include "Ballistics.h"

using namespace Plotter;
namespace
{
    Ballistics::BulletData BulletData;
    Ballistics::EnvironmentData Environment;
    Ballistics::FiringData FiringData;
    std::vector<Ballistics::TrajectoryDataPoint> G1TrajectoryDataPoints;
    std::vector<Ballistics::TrajectoryDataPoint> G7TrajectoryDataPoints;
    Curve2D::PointInfo SelectedCurvePointInfo;
    bool bCurveSelected = false;
    PlotPtr TrajectoryPlot;

    void DrawUi()
    {
        if ( !TrajectoryPlot )
        {
            TrajectoryPlot = Plot::Create();
            Curve2D Curve;
            for (size_t nQ = 0; nQ < G1TrajectoryDataPoints.size(); ++nQ)
            {
                Curve.AddPoint(G1TrajectoryDataPoints[nQ].Position.GetX(), G1TrajectoryDataPoints[nQ].Position.GetY(), nQ);
            }
            Curve.SetColor(Magenta);
            TrajectoryPlot->AddCurve(std::move(Curve), 1);
            
            for (size_t nQ = 0; nQ < G7TrajectoryDataPoints.size(); ++nQ)
            {
                Curve.AddPoint(G7TrajectoryDataPoints[nQ].Position.GetX(), G7TrajectoryDataPoints[nQ].Position.GetY(), nQ);
            }
            Curve.SetColor(Red);
            TrajectoryPlot->AddCurve(std::move(Curve), 2);
            Range2D PlotRange = TrajectoryPlot->GetExtents();
            
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
        
        DrawText(std::format("Muzzle velocity is {:.1f}m/s", FiringData.MuzzleVelocityMs), {10.f, 25.f});
        DrawText(std::format("Zero distance is {:.1f}m", FiringData.ZeroDistance), {10.f, 40.f});
        DrawText(std::format("Calibre {:.2f}mm, bullet weight {} grains", FiringData.Bullet.CallibreMm, static_cast<int>(FiringData.Bullet.MassGr)), { 200.0f, 25.0f });
        DrawText(std::format("Temperature {:.1f} Celsius", Ballistics::KelvinToCelcius(Environment.TKelvin)), {200.0f, 40.0f});
        DrawText("G7", {450.0f, 25.0f});
        DrawText("G1", {450.0f, 40.0f});
        DrawLine({{500.0f, 25.0f}, {550.0f, 25.0f}}, Red);
        DrawLine({{500.0f, 40.0f}, {550.0f, 40.0f}}, Magenta);
        
        if ( bCurveSelected )
        {
            Algebra::Vector2D Tangent = SelectedCurvePointInfo.Tangent;
            Algebra::Vector2D Normal = SelectedCurvePointInfo.Normal.Normalize();
            const float KineticEnergy = 0.5f * FiringData.Bullet.GetMassKg() * G1TrajectoryDataPoints[SelectedCurvePointInfo.MetaDataTag].Velocity.LengthSq();
            TrajectoryPlot->AddTransientLabel(std::format("x:{:.1f}m/s\ny:{:.1f}m/s\n{:.1f}J @ t:{:.001f}s",
                G1TrajectoryDataPoints[SelectedCurvePointInfo.MetaDataTag].Velocity.GetX(),
                G1TrajectoryDataPoints[SelectedCurvePointInfo.MetaDataTag].Velocity.GetY(),
                KineticEnergy,
                G1TrajectoryDataPoints[SelectedCurvePointInfo.MetaDataTag].T),
                G1TrajectoryDataPoints[SelectedCurvePointInfo.MetaDataTag].Position + 0.05f*Normal,
                DarkGray);
            TrajectoryPlot->AddTransientLine(G1TrajectoryDataPoints[SelectedCurvePointInfo.MetaDataTag].Position,
                G1TrajectoryDataPoints[SelectedCurvePointInfo.MetaDataTag].Position + Tangent, Blue);
            TrajectoryPlot->AddTransientLine(G1TrajectoryDataPoints[SelectedCurvePointInfo.MetaDataTag].Position,
                G1TrajectoryDataPoints[SelectedCurvePointInfo.MetaDataTag].Position + 0.05f*Normal, Blue);
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
        BulletData.G1BC = 0.29f;
        BulletData.G7BC = 0.275f;
        BulletData.CallibreMm = Ballistics::Callibre308Mm;

        Environment.Gravity = -9.81f;
        Environment.TKelvin = 292.0f;
        Environment.AirPressure = 101325.0f;
        Environment.UpdateAirDensityFromTandP();
    
        FiringData.Bullet = BulletData;
        FiringData.Height = 1.0f;
        FiringData.ZeroDistance = 200.0f;
        FiringData.MuzzleVelocityMs = 871.42f;
        // roughly one inch at 100m etc
        const float ToleranceM = (2.0f * (FiringData.ZeroDistance * 0.01f)) / 100.0f;

        FiringData.ZeroIn(Ballistics::G1, ToleranceM, Environment);
        Ballistics::SolverParams Solver;
        Solver.MaxTime = 10.0f;
        Solver.TimeStep = 0.01f;
        Solver.MaxX = 300.0f;
        Ballistics::SolveTrajectory(Ballistics::G1, G1TrajectoryDataPoints, FiringData, Environment, Solver);

        FiringData.ZeroIn(Ballistics::G7, ToleranceM, Environment);
        Solver.MaxTime = 10.0f;
        Solver.TimeStep = 0.01f;
        Solver.MaxX = 300.0f;
        Ballistics::SolveTrajectory(Ballistics::G7, G7TrajectoryDataPoints, FiringData, Environment, Solver);
    }

}

int main(int /*argc*/, char** /*argv*/)
{
    Solve();
#ifdef WITH_SDL
    Application::SetAppUpdateDelegate([]()
    {
        DrawUi();
    });
    Application::SetMouseMoveDelegate([](const Algebra::Vector2D& Point)
    {
        PlotPtr Plot = ViewportPointInPlot(Point, 1, [](const Curve2D::PointInfo& PointInfo)
        {
            SelectedCurvePointInfo = PointInfo;
            bCurveSelected = true;
        });
    });
    if ( Application::Init() )
    {
        Application::Run();
        //Application::Shutdown();
    }
#endif
    return 0;
}
