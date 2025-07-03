#pragma once
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

namespace Renderer
{
    class PlotRenderer;
}

namespace Plotter
{
    /**
     * @struct Point2D
     * @brief Represents a 2D point in Cartesian coordinates.
     *
     * This structure defines a point in a 2D space using two float values: `x` and `y`.
     * It provides basic operations for point arithmetic, such as addition and subtraction.
     *
     * The default constructor initializes the point to `(0, 0)`. It also allows construction
     * from two float values. Arithmetic and compound assignment operators are provided to
     * facilitate computation involving 2D points.
     */
    struct Point2D
    {
        float x;
        float y;

        constexpr Point2D() = default;
        constexpr Point2D(float x, float y) : x(x), y(y) {}

        Point2D& operator+=(const Point2D& rhs)
        {
            x += rhs.x;
            y += rhs.y;
            return *this;
        }

        Point2D operator-=(const Point2D& rhs)
        {
            x -= rhs.x;
            y -= rhs.y;
            return *this;
        }
        
        friend Point2D operator+(const Point2D& lhs, const Point2D& rhs)
        {
            return {lhs.x + rhs.x, lhs.y + rhs.y};
        }

        friend Point2D operator-(const Point2D& lhs, const Point2D& rhs)
        {
            return {lhs.x - rhs.x, lhs.y - rhs.y};
        }

        float VectorLengthSq() const
        {
            return (x*x) + (y*y);
        }
    };

    /**
     * @struct Range2D
     * @brief Represents a 2D range defined by minimum and maximum points.
     *
     * This structure is designed to manage a rectangular region in a 2D space using two points:
     * minimum (`Min`) and maximum (`Max`). The range can be empty, non-empty, or updated dynamically,
     * and provides operations for intersection and union of ranges.
     */
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

        bool IsPointInside(const Point2D& Point) const
        {
            return Point.x >= Min.x && Point.y >= Min.y && Point.x < Max.x && Point.y < Max.y;
        }
    };
    constexpr Range2D EmptyRange2D = {{std::numeric_limits<float>::max(), std::numeric_limits<float>::max()}, {std::numeric_limits<float>::min(), std::numeric_limits<float>::min()}};

    /**
     * @struct ColorRGB
     * @brief Represents a color in the RGB color space using 8-bit channels.    
     */
    struct ColorRGB
    {
        uint8_t R;
        uint8_t G;
        uint8_t B;
        ColorRGB()
            : R(0), G(0), B(0)
        {}
        constexpr explicit ColorRGB(uint8_t r,uint8_t g,uint8_t b)
            : R(r), G(g), B(b)
        {}
    };

    constexpr ColorRGB Black(0,0,0);
    constexpr ColorRGB White(255,255,255);
    constexpr ColorRGB Red(255,0,0);
    constexpr ColorRGB Green(0,255,0);
    constexpr ColorRGB Blue(0,0,255);
    constexpr ColorRGB Yellow(255,255,0);
    constexpr ColorRGB Magenta(255,0,255);
    constexpr ColorRGB Cyan(0,255,255);
    constexpr ColorRGB Gray(128,128,128);
    constexpr ColorRGB DarkGray(64,64,64);

    /**
     * @struct Label2D
     * @brief Represents a 2D label consisting of a string and its position.
     */
    struct Label2D
    {
        std::string String;
        Point2D Position;    
    };

    /**
     * @struct Line2D
     * @brief Represents a 2D line defined by its start and end points.
     */
    struct Line2D
    {
        Point2D Start;
        Point2D End;   
    };

    /**
     * @struct Curve2D
     * @brief Represents a 2D curve defined by a collection of points, color, and its extents.
     *
     * This structure provides a mechanism for defining and managing a 2D curve. It consists of a set of
     * points in 2D space, the bounding box (extents) of the curve, and its display color. The curve can
     * be dynamically updated by adding more points, with the extents being adjusted automatically.
     *
     */
    struct Curve2D
    {
        std::vector<Point2D> Points;
        Range2D Extents;
        ColorRGB Color;

        Curve2D()
        {
            Extents = EmptyRange2D;
            Color = Black;
        }

        void SetColor(ColorRGB InColor)
        {
            Color = InColor;
        }
        
        void AddPoint(float x, float y)
        {
            Points.push_back({x,y});
            Extents.Update(x,y);
        }
    };

    class Plot;
    using PlotPtr = std::shared_ptr<Plot>;

    /**
     * @class Plot
     * @brief Represents a 2D plot capable of managing curves, labels, and lines for rendering.
     *
     * This class provides functionalities to create and manage a 2D plot consisting of curves, labels, and
     * lines. It supports operations such as adding elements to the plot, checking if the plot contains any
     * data, and retrieving the extents of the plotted region. An instance of this class manages its own data
     * (including lines and labels) and can be submitted for rendering as a single unit
     */
    class Plot
    {
        std::vector<Curve2D> Curves;
        std::vector<std::pair<Label2D, ColorRGB>> Labels;
        std::vector<std::pair<Line2D, ColorRGB>> Lines;
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

        void AddLabel(const std::string& String, const Point2D& Position, ColorRGB Color=Black)
        {
            Labels.push_back({{String, Position},Color});
        }

        void AddLine(const Point2D& Start, const Point2D& End, ColorRGB Color=Black)
        {
            Lines.push_back({{Start, End},Color});
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

    /**
     * @struct IRenderer
     * @brief Abstract interface for rendering 2D graphics.
     *
     * This interface defines a contract for rendering 2D graphical elements, including
     * lines, text, and viewport management. Any concrete renderer implementation must
     * provide definitions for these methods to perform specific rendering tasks.
     *
     * The following functionalities are included in the interface:
     * - Rendering 2D lines with specified coordinates and color.
     * - Rendering textual elements at a given position with a specific color.
     * - Retrieving the extent of the current rendering viewport.
     */
    struct IRenderer
    {
        virtual ~IRenderer() = default;
        virtual void DrawLine(float x0, float y0, float x1, float y1, ColorRGB Color) = 0;
        virtual void DrawText(const std::string& Text, const Point2D& Position, ColorRGB Color) = 0;
        virtual Range2D GetViewportExtents() = 0;
    };
    using RendererPtr = std::shared_ptr<IRenderer>;
    void SetRenderer(RendererPtr InRenderer);
    RendererPtr GetRenderer();
    
    void ClearPlots();
    void DrawPlot(PlotPtr InPlot, const Range2D& ViewportWindow = EmptyRange2D);
    void DrawLine(const Line2D& Line,ColorRGB Color=Black);
    void DrawText(const std::string& Text, const Point2D& Position, ColorRGB Color=Black);

    void BeginFrame();
    void RenderFrame();
    void EndFrame();
    
    Range2D GetPlotRange();
    PlotPtr ViewportPointInPlot(const Point2D& ViewportPosition);
    
}
