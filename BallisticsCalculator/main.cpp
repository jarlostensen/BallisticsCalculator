#include <algorithm>
#include <iostream>
#include <iomanip>

#ifdef WITH_SDL
#define SDL_MAIN_USE_CALLBACKS 1  /* use the callbacks instead of main() */
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <string>
#endif

#include "BallisticsCalculator.h"
#include "Curves.h"

namespace
{
#ifdef WITH_SDL
    SDL_Window* window = NULL;
    SDL_Renderer* renderer = NULL;
    
    struct Point2D
    {
        float x;
        float y;

        constexpr Point2D() = default;
        constexpr Point2D(float x, float y) : x(x), y(y) {}

        friend Point2D operator+(const Point2D& lhs, const Point2D& rhs)
        {
            return {lhs.x + rhs.x, lhs.y + rhs.y};
        }

        friend Point2D operator-(const Point2D& lhs, const Point2D& rhs)
        {
            return {lhs.x - rhs.x, lhs.y - rhs.y};
        }
    };

    struct Range2D
    {
        Point2D Min;
        Point2D Max;

        constexpr Range2D() = default;
        constexpr Range2D(Point2D min, Point2D max) : Min(min), Max(max) {}

        void Update(float x, float y)
        {
            Min.x = std::min(Min.x, x);
            Min.y = std::min(Min.y, y);
            Max.x = std::max(Max.x, x);
            Max.y = std::max(Max.y, y);
        }

        bool IsNonEmpty() const
        {
            return Min.x < Max.x && Min.y < Max.y;
        }

        float Width() const
        {
            return Max.x - Min.x;
        }

        float Height() const
        {
            return Max.y - Min.y;
        }
    };

    using PointType2d = Point2D;
    using LineType2d = std::pair<PointType2d, PointType2d>;
    std::vector<LineType2d> LineBuffer;

    constexpr Range2D ViewportMargins{{2.0f,2.0f}, {2.0f,2.0f}};
    Range2D ViewportExtents = {Point2D(0.0f,0.0f) + ViewportMargins.Min, Point2D(800.0f,600.0f) - ViewportMargins.Max};
    
    void UpdateViewportExtents()
    {
        int ViewPortWidth;
        int ViewPortHeight;
        SDL_GetRenderOutputSize(renderer, &ViewPortWidth, &ViewPortHeight);
        ViewportExtents.Min.x = 0.0f;
        ViewportExtents.Min.y = 0.0f;
        ViewportExtents.Max.x = static_cast<float>(ViewPortWidth);
        ViewportExtents.Max.y = static_cast<float>(ViewPortHeight);
    }

    void DrawGrid()
    {
        SDL_RenderLine(renderer,
        0, ViewportExtents.Height()/2.0f,
        ViewportExtents.Width(), ViewportExtents.Height()/2.0f);

        SDL_RenderLine(renderer,
            ViewportExtents.Width()/2.0f, 0,
            ViewportExtents.Width()/2.0f, ViewportExtents.Height());
    }
    
    void GenerateTransforms(const Range2D& WindowExtents, PointType2d& OutScaleTransform, PointType2d& OutNormalizationTransform)
    {
        OutScaleTransform.x = (ViewportExtents.Max.x - ViewportExtents.Min.x) / (WindowExtents.Max.x - WindowExtents.Min.x);
        OutScaleTransform.y = (ViewportExtents.Max.y - ViewportExtents.Min.y) / (WindowExtents.Max.y - WindowExtents.Min.y);
        OutNormalizationTransform.x = (WindowExtents.Max.x * ViewportExtents.Min.x - WindowExtents.Min.x * ViewportExtents.Max.x) / (WindowExtents.Max.x - WindowExtents.Min.x);
        OutNormalizationTransform.y = (WindowExtents.Max.y * ViewportExtents.Min.y - WindowExtents.Min.y * ViewportExtents.Max.y) / (WindowExtents.Max.y - WindowExtents.Min.y);
    }
    
    void ViewportTransform(const PointType2d& ScaleTransform, const PointType2d& NormalizationTransform,
        const std::vector<PointType2d>& Points, std::vector<PointType2d>& OutPoints)
    {
        OutPoints.resize(Points.size());
        for (size_t n = 0; n < Points.size(); ++n)
        {
            OutPoints[n].x = Points[n].x * ScaleTransform.x + NormalizationTransform.x;
            OutPoints[n].y = ViewportExtents.Max.y - (Points[n].y * ScaleTransform.y + NormalizationTransform.y);
        }
    }
    
    struct Curve2D
    {
        std::vector<Point2D> Points;
        Range2D Extents;

        Curve2D()
        {
            Extents.Min = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
            Extents.Max = {std::numeric_limits<float>::min(), std::numeric_limits<float>::min()};    
        }

        bool IsEmpty() const
        {
            return Points.empty() && Extents.IsNonEmpty();
        }

        bool HasValidExtents() const
        {
            return Extents.IsNonEmpty();
        }
        
        void AddPoint(float x, float y)
        {
            Points.push_back({x,y});
            Extents.Update(x,y);
        }
    };
    using CurveBufferType = std::vector<Curve2D>;
    CurveBufferType CurveBuffer;

    float PlotTransformY(float y)
    {
        return y;
    }

    float PlotTransformX(float x)
    {
        return x;
    }

    void PlotLine(float x0, float y0, float x1, float y1)
    {
        LineBuffer.push_back({
            {PlotTransformX(x0),PlotTransformY(y0)},
            {PlotTransformX(x1),PlotTransformY(y1)}
        });
    }

    void PlotCurve(const Curve2D& Curve)
    {
        CurveBuffer.push_back(Curve);
    }

    void ClearCurves()
    {
        CurveBuffer.clear();
    }

    void DrawCurves()
    {
        for (auto& Curve : CurveBuffer)
        {
            std::vector<Point2D> TransformedPoints;
            PointType2d ScaleTransform;
            PointType2d NormalizationTransform;
            GenerateTransforms(Curve.Extents, ScaleTransform, NormalizationTransform);
            ViewportTransform(ScaleTransform, NormalizationTransform, Curve.Points, TransformedPoints);

            Point2D& PrevPoint = TransformedPoints[0];
            for (size_t n = 1; n < TransformedPoints.size(); ++n)
            {
                SDL_RenderLine(renderer,
                               PrevPoint.x,
                               PrevPoint.y,
                               TransformedPoints[n].x,
                               TransformedPoints[n].y);
                PrevPoint = TransformedPoints[n];
            }
        }
    }
    
#endif

    Ballistics::BulletData BulletData;
    Ballistics::EnvironmentData Environment;
    Ballistics::FiringData FiringData;
    std::vector<Ballistics::TrajectoryDataPoint> TrajectoryDataPoints;
    float MaxX = std::numeric_limits<float>::min();
    float MaxY = std::numeric_limits<float>::min();
    float MinX = std::numeric_limits<float>::max();
    float MinY = std::numeric_limits<float>::max();

    void DrawTrajectory(bool ClearCurrent)
    {
#if WITH_SDL
        if (ClearCurrent)
        {
            ClearCurves();   
        }
        Curve2D Curve;
        for (size_t nQ = 1; nQ < TrajectoryDataPoints.size(); ++nQ)
        {
            Curve.AddPoint(TrajectoryDataPoints[nQ].DistanceX, TrajectoryDataPoints[nQ].DistanceY);    
        }
        PlotCurve(Curve);
#else
        std::cout << "T\tX\tY\tVx\n";
        for (const Ballistics::TrajectoryDataPoint& Q : TrajectoryDataPoints)
        {
            const float KineticEnergy = 0.5f * BulletData.GetMassKg() * (Q.VelocityX * Q.VelocityX);
            std::cout << std::fixed << std::setprecision(2) << Q.T << "\t" << Q.DistanceX << "\t" << Q.DistanceY*100.0f << " cm\t" << Q.VelocityX << " m/s (" << (Q.VelocityX * Ballistics::MsToFtS) << " ft/s)\t" << KineticEnergy << " Joules\n";
        }
#endif
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
        FiringData.Height = 10.0f;
        FiringData.ZeroDistance = 100.0f;
        FiringData.ZeroIn(0.01f, Environment);

        Ballistics::SolverParams Solver;
        Solver.MaxTime = 10.0f;
        Solver.TimeStep = 0.01f;
        Solver.MaxX = 300.0f;
        
        Ballistics::SolveTrajectoryG7(TrajectoryDataPoints, FiringData, Environment, Solver);

        for (const Ballistics::TrajectoryDataPoint& Q : TrajectoryDataPoints)
        {
            MaxX = std::max(Q.DistanceX, MaxX);
            MaxY = std::max(Q.DistanceY, MaxY);
            MinX = std::min(Q.DistanceX, MinX);
            MinY = std::min(Q.DistanceY, MinY);
        }
        DrawTrajectory(false);
    }
}

#ifndef WITH_SDL
int main(int /*argc*/, char** /*argv*/)
{
    Solve();
    return 0;
}
#endif

#ifdef WITH_SDL

SDL_AppResult SDL_AppInit(void** /*appstate*/, int /*argc*/, char* argv[])
{
    (void)argv;
    if (!SDL_CreateWindowAndRenderer("Ballistics Calculator", 800, 600, SDL_WINDOW_RESIZABLE, &window, &renderer)) {
        SDL_Log("Couldn't create window and renderer: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }

    Solve();

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);
    
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void* /*appstate*/, SDL_Event* event)
{
    if (event->type == SDL_EVENT_KEY_DOWN ||
        event->type == SDL_EVENT_QUIT) {
        return SDL_APP_SUCCESS;
    }
    if (event->type == SDL_EVENT_WINDOW_RESIZED)
    {
        UpdateViewportExtents();
        DrawTrajectory(true);
    }
    return SDL_APP_CONTINUE;
}


SDL_AppResult SDL_AppIterate(void* /*appstate*/)
{
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);
    
    SDL_SetRenderDrawColor(renderer, 64, 64, 64, 255);
    DrawCurves();
    
    SDL_SetRenderDrawColor(renderer, 128, 128, 128, 255);
    DrawGrid();

    SDL_RenderPresent(renderer);
    
    return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* /*appstate*/, SDL_AppResult /*result*/)
{
}
#endif // #ifdef WITH_SDL