#pragma once
#include <algorithm>
#include <string>
#include <vector>

namespace Ui
{
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

        Range2D& operator&=(const Range2D& rhs)
        {
            Min.x = std::max(Min.x, rhs.Min.x);
            Min.y = std::max(Min.y, rhs.Min.y);
            Max.x = std::min(Max.x, rhs.Max.x);
            Max.y = std::min(Max.y, rhs.Max.y);
            return *this;
        }

        Range2D& operator|=(const Range2D& rhs)
        {
            Min.x = std::min(Min.x, rhs.Min.x);
            Min.y = std::min(Min.y, rhs.Min.y);
            Max.x = std::max(Max.x, rhs.Max.x);
            Max.y = std::max(Max.y, rhs.Max.y);
            return *this;
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
    constexpr Range2D EmptyRange2D = {{std::numeric_limits<float>::max(), std::numeric_limits<float>::max()}, {std::numeric_limits<float>::min(), std::numeric_limits<float>::min()}};

    struct Curve2D
    {
        std::vector<Point2D> Points;
        Range2D Extents;

        Curve2D()
        {
            Extents = EmptyRange2D;    
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

    using PointType2d = Point2D;
    using LineType2d = std::pair<PointType2d, PointType2d>;
    
    void UpdateViewportExtents();
    void GenerateTransforms(const Range2D& CurveDataExtents, PointType2d& OutScaleTransform, PointType2d& OutNormalizationTransform);
    void ViewportTransform(const PointType2d& ScaleTransform, const PointType2d& NormalizationTransform,
        const std::vector<PointType2d>& Points, std::vector<PointType2d>& OutPoints);

    void PlotLine(float x0, float y0, float x1, float y1);
    void ClearLines();
    void PlotCurve(const Curve2D& Curve);
    void ClearCurves();
    void PlotText(const std::string& Text, const PointType2d& Position);

    Range2D GetMaximalDataRange();
    
}

extern void AppInit();
extern void AppUpdate();
