#include "Ui.h"

#include <cassert>

namespace 
{
    Ui::RendererPtr RendererImpl;
}

namespace Ui
{
    using LineBufferType = std::vector<Line2D>;
    using CurveBufferType = std::vector<Curve2D>;
    using TextBufferType = std::vector<Label2D>;
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
    
    void DrawPlot(PlotPtr InPlot, const Range2D& ViewportWindow)
    {
        PlotBuffer.emplace_back(InPlot, ViewportWindow);
    }

    void DrawLine(const Line2D& Line)
    {
        LineBuffer.push_back(Line);
    }

    void DrawText(const std::string& Text, const Point2D& Position)
    {
        TextBuffer.push_back({Text, Position});
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
}

using namespace Ui;
namespace Renderer
{
    static void RenderFilledCircle(float centerX, float centerY, float radius)
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
                centerX + x, centerY + y);
            RendererImpl->DrawLine( 
                centerX - x, centerY - y, 
                centerX + x, centerY - y);
            RendererImpl->DrawLine( 
                centerX - y, centerY + x, 
                centerX + y, centerY + x);
            RendererImpl->DrawLine( 
                centerX - y, centerY - x, 
                centerX + y, centerY - x);

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
                                       TransformedPoints[n].y);
                        RenderFilledCircle(TransformedPoints[n].x, TransformedPoints[n].y, 2.0f);
                        PrevPoint = TransformedPoints[n];
                    }
                    RenderFilledCircle(PrevPoint.x, PrevPoint.y, 2.0f);
                }
                
                for (const auto & Line : Plot.first->Lines)
                {
                    Line2D TransformedLine;
                    ToViewport(Transform, ViewportWindowExtents, Line, TransformedLine);
                    RendererImpl->DrawLine(
                        TransformedLine.Start.x,
                        TransformedLine.Start.y,
                        TransformedLine.End.x,
                        TransformedLine.End.y);
                }

                for (const auto & Label : Plot.first->Labels)
                {
                    Point2D TransformedLabelPosition;
                    ToViewport(Transform,ViewportWindowExtents, Label.Position, TransformedLabelPosition);
                    RendererImpl->DrawText(Label.String, TransformedLabelPosition);
                }
            }
        }
    };
}

namespace Ui
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
            RendererImpl->DrawLine(Line.Start.x, Line.Start.y, Line.End.x, Line.End.y);
        }

        for (const auto & Text : TextBuffer)
        {
            RendererImpl->DrawText(Text.String, Text.Position);
        }
    }

    void EndFrame()
    {
        assert(bInFrame);
        bInFrame = false;
    }
}
