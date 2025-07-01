#pragma once
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

namespace Renderer
{
    class PlotRenderer;
}

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

        bool IsEmpty() const
        {
            return Min.x > Max.x || Min.y > Max.y;
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

        Range2D& operator|=(const Point2D& rhs)
        {
            Min.x = std::min(Min.x, rhs.x);
            Min.y = std::min(Min.y, rhs.y);
            Max.x = std::max(Max.x, rhs.x);
            Max.y = std::max(Max.y, rhs.y);
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

    struct Label2D
    {
        std::string String;
        Point2D Position;    
    };
    struct Line2D
    {
        Point2D Start;
        Point2D End;   
    };
    
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

    class Plot;
    using PlotPtr = std::shared_ptr<Plot>;
    class Plot
    {
        std::vector<Curve2D> Curves;
        std::vector<Label2D> Labels;
        std::vector<Line2D> Lines;
        Range2D Extents;

        struct PlotPrivate
        {
            explicit PlotPrivate() = default;
        };
        
        friend class Renderer::PlotRenderer;
    public:
        Plot(PlotPrivate&&)
        {
            Extents = EmptyRange2D;
        }
        
        static PlotPtr Create()
        {
            return std::make_shared<Plot>(PlotPrivate{});
        }

        void AddCurve(const Curve2D& Curve)
        {
            Curves.push_back(Curve);
            Extents |= Curve.Extents;
        }

        void AddLabel(const std::string& String, const Point2D& Position)
        {
            Labels.push_back({String, Position});
            //Extents |= Position;
        }

        void AddLine(const Point2D& Start, const Point2D& End)
        {
            Lines.push_back({Start, End});
            //Extents |= {Start, End};
        }

        bool IsEmpty() const
        {
            return Extents.IsEmpty();
        }

        Range2D GetExtents() const
        {
            return Extents;
        }
    };

    void DrawLine(float x0, float y0, float x1, float y1);
    void DrawText(const std::string& Text, const Point2D& Position);

    void ClearPlots();
    void AddPlot(PlotPtr InPlot);
    
    Range2D GetPlotRange();
    
}

extern void AppInit();
extern void AppUpdate();
