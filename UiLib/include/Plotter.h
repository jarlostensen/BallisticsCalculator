﻿#pragma once
#include <algorithm>
#include <functional>
#include <string>
#include <vector>
#include <memory>
#include "Algebra.h"
#include "Curves.h"

namespace Renderer
{
    class PlotRenderer;
}

namespace Plotter
{
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
        Algebra::Vector2D Min;
        Algebra::Vector2D Max;

        constexpr Range2D() = default;
        constexpr Range2D(Algebra::Vector2D min, Algebra::Vector2D max) : Min(min), Max(max) {}
        
        void Update(float x, float y)
        {
            Min.SetX(std::min(Min.GetX(), x));
            Min.SetY(std::min(Min.GetY(), y));
            Max.SetX(std::max(Max.GetX(), x));
            Max.SetY(std::max(Max.GetY(), y));
        }

        bool IsNonEmpty() const
        {
            return Min.GetX() < Max.GetX() && Min.GetY() < Max.GetY();
        }

        bool IsEmpty() const
        {
            return Min.GetX() > Max.GetX() || Min.GetY() > Max.GetY();
        }

        Range2D& operator&=(const Range2D& rhs)
        {
            Min.SetX(std::max(Min.GetX(), rhs.Min.GetX()));
            Min.SetY(std::max(Min.GetY(), rhs.Min.GetY()));
            Max.SetX(std::min(Max.GetX(), rhs.Max.GetX()));
            Max.SetY(std::min(Max.GetY(), rhs.Max.GetY()));
            return *this;
        }

        Range2D& operator|=(const Range2D& rhs)
        {
            Min.SetX(std::min(Min.GetX(), rhs.Min.GetX()));
            Min.SetY(std::min(Min.GetY(), rhs.Min.GetY()));
            Max.SetX(std::max(Max.GetX(), rhs.Max.GetX()));
            Max.SetY(std::max(Max.GetY(), rhs.Max.GetY()));
            return *this;
        }

        Range2D& operator|=(const Algebra::Vector2D& rhs)
        {
            Min.SetX(std::min(Min.GetX(), rhs.GetX()));
            Min.SetY(std::min(Min.GetY(), rhs.GetY()));
            Max.SetX(std::max(Max.GetX(), rhs.GetX()));
            Max.SetY(std::max(Max.GetY(), rhs.GetY()));
            return *this;
        }
        
        float Width() const
        {
            return Max.GetX() - Min.GetX();
        }

        float Height() const
        {
            return Max.GetY() - Min.GetY();
        }

        bool IsPointInside(const Algebra::Vector2D& Point) const
        {
            return Point.GetX() >= Min.GetX() && Point.GetY() >= Min.GetY() && Point.GetX() < Max.GetX() && Point.GetY() < Max.GetY();
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
        Algebra::Vector2D Position;
    };

    /**
     * @struct Line2D
     * @brief Represents a 2D line defined by its start and end points.
     */
    struct Line2D
    {
        Algebra::Vector2D Start;
        Algebra::Vector2D End;
    };

    class Plot;
    using PlotPtr = std::shared_ptr<Plot>;
    using MetaDataTagType = uintptr_t;
    constexpr MetaDataTagType NullMetaDataTag = std::numeric_limits<MetaDataTagType>::max();
    
    /**
     * @struct Curve2D
     * @brief Represents a 2D curve defined by a collection of points, color, and its extents.
     *
     * This structure provides a mechanism for defining and managing a 2D curve with meta data attached to each point
     *
     */
    class Curve2D
    {
    public:
        Curve2D()
        {
            Extents = EmptyRange2D;
            Color = Black;
        }

        void SetColor(ColorRGB InColor)
        {
            Color = InColor;
        }

        const ColorRGB& GetColor() const
        {
            return Color;
        }
        
        void AddPoint(float x, float y, MetaDataTagType MetaDataTag=0)
        {
            Points.emplace_back(x,y);
            PointMetaTags.push_back(MetaDataTag);
            Extents.Update(x,y);
        }

        void AddPoint(const Algebra::Vector2D& Point, MetaDataTagType MetaDataTag = NullMetaDataTag)
        {
            Points.emplace_back(Point);
            PointMetaTags.push_back(MetaDataTag);
            Extents.Update(Point.GetX(), Point.GetY());
        }

        
        /**
         * @struct PointInfo
         * @brief Represents detailed information about a point in a 2D curve.
         *
         * This structure contains data for a specific point on a 2D curve contained in the plot, including its position,
         * normal vector, tangent vector, and associated metadata
         */
        struct PointInfo
        {
            Algebra::Vector2D Point;
            Algebra::Vector2D Normal;
            Algebra::Vector2D Tangent;
            MetaDataTagType MetaDataTag;
        };

        struct PointIterInfo
        {
            Algebra::Vector2D Point;
            MetaDataTagType MetaDataTag;
        };

        class Iterator
        {
            friend class Curve2D;
            Iterator(const std::vector<Algebra::Vector2D>& points, const std::vector<MetaDataTagType>& metadata, size_t pos)
                : Points(&points)
                , MetaData(&metadata)
                , nPos(pos)
            {
                Current.Point = (*Points)[nPos];
                Current.MetaDataTag = (*MetaData)[nPos];
            }
        public:
            using iterator_category = std::forward_iterator_tag;
            using value_type = PointInfo;
            using difference_type = std::ptrdiff_t;
            using pointer = PointInfo*;
            using reference = PointInfo&;

            Iterator() = default;
            Iterator(const Iterator& Rhs) = default;
            
            Iterator& operator++()
            {
                ++nPos;
                if (nPos < (*Points).size())
                {
                    Current.Point = (*Points)[nPos];
                    Current.MetaDataTag = (*MetaData)[nPos];
                }
                return *this;
            }

            Iterator operator++(int)
            {
                Iterator tmp = *this;
                ++(*this);
                return tmp;
            }

            bool operator==(const Iterator& other) const
            {
                return nPos == other.nPos && Points == other.Points;
            }

            bool operator!=(const Iterator& other) const {
                return !(*this == other);
            }

            const PointIterInfo& operator*() const {
                return Current;
            }

            const PointIterInfo* operator->() const {
                return &Current;
            }

        private:
            const std::vector<Algebra::Vector2D>* Points;
            const std::vector<MetaDataTagType>* MetaData;
            size_t nPos;
            PointIterInfo Current;
        };

        Iterator begin() const
        {
            return {Points, PointMetaTags, 0};
        }

        Iterator end() const
        {
            return {Points, PointMetaTags, Points.size()};
        }

        Iterator Find(MetaDataTagType MetaDataTag) const
        {
            for (size_t nT = 0; nT < Points.size(); nT++)
            {
                if (PointMetaTags[nT] == MetaDataTag)
                {
                    return {Points, PointMetaTags, nT};
                }
            }
            return end();
        }

        std::pair<Iterator, float> FindNearest(const Algebra::Vector2D& Point) const
        {
            float MinDistanceSq = std::numeric_limits<float>::max();
            size_t MinIndex = 0;
            for (size_t nT = 0; nT < Points.size(); nT++)
            {
                const float DistanceSq = (Point - Points[nT]).LengthSq();
                if (DistanceSq < MinDistanceSq)
                {
                    MinDistanceSq = DistanceSq;
                    MinIndex = nT;
                }
            }
            return {{Points, PointMetaTags, MinIndex}, MinDistanceSq};
        }

        static void GetPointInfo(const Iterator& Iter, PointInfo& OutPointInfo);

    private:
        std::vector<Algebra::Vector2D> Points;
        std::vector<MetaDataTagType> PointMetaTags;
        Range2D Extents;
        ColorRGB Color;
        friend class Plot;
        friend class Renderer::PlotRenderer;
    };

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
        std::vector<MetaDataTagType> CurveMetaTags;
        std::vector<std::pair<Label2D, ColorRGB>> Labels;
        std::vector<std::pair<Line2D, ColorRGB>> Lines;
        struct
        {
            std::vector<std::pair<Label2D, ColorRGB>> Labels;
            std::vector<std::pair<Line2D, ColorRGB>> Lines;
            void Clear()
            {
                Labels.clear();
                Lines.clear();
            }
            operator bool() const
            {
                return !Labels.empty() || !Lines.empty();
            }
        } TransientElements;
        
        Range2D Extents;

        struct PlotPrivate
        {
            explicit PlotPrivate() = default;
        };
        
        friend class Renderer::PlotRenderer;
    public:
        Plot(PlotPrivate&&)
        {
            Curves.reserve(2);
            CurveMetaTags.reserve(2);
            Extents = EmptyRange2D;
        }
        
        static PlotPtr Create()
        {
            return std::make_shared<Plot>(PlotPrivate{});
        }

        void AddCurve(const Curve2D& Curve, MetaDataTagType MetaDataTag=NullMetaDataTag)
        {
            Curves.push_back(Curve);
            CurveMetaTags.push_back(MetaDataTag);
            Extents |= Curve.Extents;
        }

        void AddCurve(Curve2D&& Curve, MetaDataTagType MetaDataTag=NullMetaDataTag)
        {
            Extents |= Curve.Extents;
            Curves.push_back(std::forward<Curve2D>(Curve));
            CurveMetaTags.push_back(MetaDataTag);
        }

        void AddLabel(const std::string& String, const Algebra::Vector2D& Position, ColorRGB Color=Black)
        {
            Labels.push_back({{String, Position},Color});
        }

        void AddTransientLabel(const std::string& String, const Algebra::Vector2D& Position, ColorRGB Color=Black)
        {
            TransientElements.Labels.push_back({{String, Position},Color});
        }

        void AddTransientLine(const Algebra::Vector2D& Start, const Algebra::Vector2D& End, ColorRGB Color = Black)
        {
            TransientElements.Lines.push_back({ {Start, End},Color });
        }

        void AddLine(const Algebra::Vector2D& Start, const Algebra::Vector2D& End, ColorRGB Color=Black)
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

        std::optional<Curve2D::Iterator> FindNearest(const Algebra::Vector2D& Point, MetaDataTagType MetaDataTagFilter) const;
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
        virtual void DrawText(const std::string& Text, const Algebra::Vector2D& Position, ColorRGB Color) = 0;
        virtual Range2D GetViewportExtents() = 0;
    };
    using RendererPtr = std::shared_ptr<IRenderer>;
    void SetRenderer(RendererPtr InRenderer);
    RendererPtr GetRenderer();
    
    void ClearPlots();
    void DrawPlot(PlotPtr InPlot, const Range2D& ViewportWindow = EmptyRange2D);
    void DrawLine(const Line2D& Line,ColorRGB Color=Black);
    void DrawText(const std::string& Text, const Algebra::Vector2D& Position, ColorRGB Color=Black);

    void BeginFrame();
    void RenderFrame();
    void EndFrame();
    
    Range2D GetPlotRange();

    using ViewportPointInPlotDelegateType = std::function<void(const Curve2D::PointInfo& PointInfo)>;
    PlotPtr ViewportPointInPlot(const Algebra::Vector2D& ViewportPosition, MetaDataTagType MetaDataTag, ViewportPointInPlotDelegateType&& ViewportPointInPlotDelegate);
    
}
