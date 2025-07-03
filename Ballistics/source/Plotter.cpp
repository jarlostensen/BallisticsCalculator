#include "Plotter.h"

#include <cassert>

#include "Curves.h"

namespace 
{
    Plotter::RendererPtr RendererImpl;
}

namespace Plotter
{
    using LineBufferType = std::vector<std::pair<Line2D, ColorRGB>>;
    using CurveBufferType = std::vector<Curve2D>;
    using TextBufferType = std::vector<std::pair<Label2D, ColorRGB>>;
    using PlotBufferType = std::vector<std::pair<PlotPtr, Range2D>>;

    LineBufferType LineBuffer;
    TextBufferType TextBuffer;
    PlotBufferType PlotBuffer;

    struct ViewportTransform
    {
        Point2D Scale;
        Point2D Translation;
    };
    
    Range2D MaximalDataRange = EmptyRange2D;

    void GenerateTransform(const Range2D& Extents, const Range2D& InViewportExtents, ViewportTransform& OutViewportTransform)
    {
        OutViewportTransform.Scale.x = (InViewportExtents.Max.x - InViewportExtents.Min.x) / ( Extents.Max.x -  Extents.Min.x);
        OutViewportTransform.Scale.y = (InViewportExtents.Max.y - InViewportExtents.Min.y) / ( Extents.Max.y -  Extents.Min.y);
        OutViewportTransform.Translation.x = ( Extents.Max.x * InViewportExtents.Min.x -  Extents.Min.x * InViewportExtents.Max.x) / ( Extents.Max.x -  Extents.Min.x);
        OutViewportTransform.Translation.y = ( Extents.Max.y * InViewportExtents.Min.y -  Extents.Min.y * InViewportExtents.Max.y) / ( Extents.Max.y -  Extents.Min.y);
    }

    void ToViewport(const ViewportTransform& Transform, const Range2D& ViewportExtents, const std::vector<Point2D>& Points, std::vector<Point2D>& OutPoints)
    {
        OutPoints.resize(Points.size());
        for (size_t n = 0; n < Points.size(); ++n)
        {
            OutPoints[n].x = Points[n].x * Transform.Scale.x + Transform.Translation.x;
            OutPoints[n].y = ViewportExtents.Min.y + (ViewportExtents.Max.y - (Points[n].y * Transform.Scale.y + Transform.Translation.y));
        }
    }

    void ToViewport(const ViewportTransform& Transform, const Range2D& ViewportExtents, const Line2D& Line, Line2D& OutLine)
    {
        OutLine.Start = {Line.Start.x * Transform.Scale.x + Transform.Translation.x, ViewportExtents.Min.y + (ViewportExtents.Max.y -  (Line.Start.y * Transform.Scale.y + Transform.Translation.y))};
        OutLine.End  = {Line.End.x * Transform.Scale.x + Transform.Translation.x, ViewportExtents.Min.y + (ViewportExtents.Max.y - (Line.End.y * Transform.Scale.y + Transform.Translation.y))};    
    }

    void ToViewport(const ViewportTransform& Transform, const Range2D& ViewportExtents, const Point2D& Point, Point2D& OutPoint)
    {
        OutPoint.x = Point.x * Transform.Scale.x + Transform.Translation.x;
        OutPoint.y = ViewportExtents.Min.y + (ViewportExtents.Max.y - (Point.y * Transform.Scale.y + Transform.Translation.y));    
    }

    void FromViewport(const ViewportTransform& Transform, const Range2D& ViewportExtents, const Point2D& Point, Point2D& OutPoint)
    {
        OutPoint.x = (Point.x - Transform.Translation.x) / Transform.Scale.x;
        OutPoint.y = ((Point.y - ViewportExtents.Min.y - ViewportExtents.Max.y) - Transform.Translation.y)/Transform.Scale.y; 
    }
    
    void DrawPlot(PlotPtr InPlot, const Range2D& ViewportWindow)
    {
        PlotBuffer.emplace_back(InPlot, ViewportWindow);
    }

    void DrawLine(const Line2D& Line, ColorRGB Color)
    {
        LineBuffer.push_back({Line,Color});
    }

    void DrawText(const std::string& Text, const Point2D& Position, ColorRGB Color)
    {
        TextBuffer.push_back({{Text, Position},Color});
    }

    void SetRenderer(RendererPtr InRenderer)
    {
        RendererImpl = InRenderer;
    }

    RendererPtr GetRenderer()
    {
        return RendererImpl;   
    }

    void ClearPlots()
    {
        PlotBuffer.clear();
        MaximalDataRange = EmptyRange2D;   
    }

    Range2D GetPlotRange()
    {
        return MaximalDataRange;   
    }

    PlotPtr ViewportPointInPlot(const Point2D& ViewportPosition)
    {
        ViewportTransform Transform;
        Range2D ViewportWindowExtents = RendererImpl->GetViewportExtents();
        GenerateTransform(GetPlotRange(), ViewportWindowExtents, Transform);
        Point2D Position;
        FromViewport(Transform, ViewportWindowExtents, ViewportPosition, Position);
        for (const auto & Plot : PlotBuffer)
        {
            if ( Plot.first->GetExtents().IsPointInside(Position) )
            {
                return Plot.first;
            }
        }
        return {};
    }
}

using namespace Plotter;
namespace Renderer
{
    static void RenderFilledCircle(float centerX, float centerY, float radius, ColorRGB color)
    {
        // Using the midpoint circle algorithm
        const float diameter = radius * 2;
        float x = radius - 1;
        float y = 0;
        float dx = 1;
        float dy = 1;
        float error = dx - diameter;

        while (x >= y) {
            // Draw horizontal lines for each quadrant to fill the circle
            RendererImpl->DrawLine( 
                centerX - x, centerY + y, 
                centerX + x, centerY + y,
                color);
            RendererImpl->DrawLine( 
                centerX - x, centerY - y, 
                centerX + x, centerY - y,
                color);
            RendererImpl->DrawLine( 
                centerX - y, centerY + x, 
                centerX + y, centerY + x,
                color);
            RendererImpl->DrawLine( 
                centerX - y, centerY - x, 
                centerX + y, centerY - x,
                color);

            if (error <= 0) {
                y++;
                error += dy;
                dy += 2;
            }
            if (error > 0) {
                x--;
                dx += 2;
                error += dx - diameter;
            }
        }
    }

    class PlotRenderer
    {
    public:
        static void RenderPlots()
        {
            for (auto& Plot : PlotBuffer)
            {
                ViewportTransform Transform;
                Range2D ViewportWindowExtents = Plot.second.IsEmpty() ? RendererImpl->GetViewportExtents() : Plot.second;
                GenerateTransform(Plot.first->GetExtents(), ViewportWindowExtents, Transform);
                for (const auto & Curve : Plot.first->Curves)
                {
                    std::vector<Point2D> TransformedPoints;
                    ToViewport(Transform, ViewportWindowExtents, Curve.Points, TransformedPoints);
                    Point2D& PrevPoint = TransformedPoints[0];
                    for (size_t n = 1; n < TransformedPoints.size(); ++n)
                    {
                        RendererImpl->DrawLine(
                                       PrevPoint.x,
                                       PrevPoint.y,
                                       TransformedPoints[n].x,
                                       TransformedPoints[n].y,
                                       Curve.Color);
                        RenderFilledCircle(TransformedPoints[n].x, TransformedPoints[n].y, 2.0f, Curve.Color);
                        PrevPoint = TransformedPoints[n];
                    }
                    RenderFilledCircle(PrevPoint.x, PrevPoint.y, 2.0f, Curve.Color);
                }
                
                for (const auto & Line : Plot.first->Lines)
                {
                    Line2D TransformedLine;
                    ToViewport(Transform, ViewportWindowExtents, Line.first, TransformedLine);
                    RendererImpl->DrawLine(
                        TransformedLine.Start.x,
                        TransformedLine.Start.y,
                        TransformedLine.End.x,
                        TransformedLine.End.y,
                        Line.second);
                }

                for (const auto & Label : Plot.first->Labels)
                {
                    Point2D TransformedLabelPosition;
                    ToViewport(Transform,ViewportWindowExtents, Label.first.Position, TransformedLabelPosition);
                    RendererImpl->DrawText(Label.first.String, TransformedLabelPosition, Label.second);
                }
            }
        }
    };
}

namespace Plotter
{
    bool bInFrame = false;
    
    void BeginFrame()
    {
        assert(!bInFrame);
        bInFrame = true;
        LineBuffer.clear();
        TextBuffer.clear();
        PlotBuffer.clear();
        MaximalDataRange = EmptyRange2D;
    }
    
    void RenderFrame()
    {
        assert(bInFrame);
        Renderer::PlotRenderer::RenderPlots();

        for (const auto & Line : LineBuffer)
        {
            RendererImpl->DrawLine(Line.first.Start.x, Line.first.Start.y, Line.first.End.x, Line.first.End.y, Line.second);
        }

        for (const auto & Text : TextBuffer)
        {
            RendererImpl->DrawText(Text.first.String, Text.first.Position, Text.second);
        }
    }

    void EndFrame()
    {
        assert(bInFrame);
        bInFrame = false;
    }
}
