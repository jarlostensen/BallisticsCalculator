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
        Algebra::Vector2D Scale;
        Algebra::Vector2D Translation;
    };
    
    Range2D MaximalDataRange = EmptyRange2D;

    void GenerateTransform(const Range2D& Extents, const Range2D& InViewportExtents, ViewportTransform& OutViewportTransform)
    {
        OutViewportTransform.Scale.SetX( (InViewportExtents.Max.GetX() - InViewportExtents.Min.GetX()) / ( Extents.Max.GetX() -  Extents.Min.GetX()) );
        OutViewportTransform.Scale.SetY( (InViewportExtents.Max.GetY() - InViewportExtents.Min.GetY()) / ( Extents.Max.GetY() -  Extents.Min.GetY()) );
        OutViewportTransform.Translation.SetX( ( Extents.Max.GetX() * InViewportExtents.Min.GetX() -  Extents.Min.GetX() * InViewportExtents.Max.GetX()) / ( Extents.Max.GetX() -  Extents.Min.GetX()) );
        OutViewportTransform.Translation.SetY( ( Extents.Max.GetY() * InViewportExtents.Min.GetY() -  Extents.Min.GetY() * InViewportExtents.Max.GetY()) / ( Extents.Max.GetY() -  Extents.Min.GetY()) );
    }

    void ToViewport(const ViewportTransform& Transform, const Range2D& ViewportExtents, const std::vector<Algebra::Vector2D>& Points, std::vector<Algebra::Vector2D>& OutPoints)
    {
        OutPoints.resize(Points.size());
        for (size_t n = 0; n < Points.size(); ++n)
        {
            OutPoints[n].SetX( Points[n].GetX() * Transform.Scale.GetX() + Transform.Translation.GetX() );
            OutPoints[n].SetY( ViewportExtents.Min.GetY() + (ViewportExtents.Max.GetY() - (Points[n].GetY() * Transform.Scale.GetY() + Transform.Translation.GetY())) );
        }
    }

    void ToViewport(const ViewportTransform& Transform, const Range2D& ViewportExtents, const Line2D& Line, Line2D& OutLine)
    {
        OutLine.Start = {Line.Start.GetX() * Transform.Scale.GetX() + Transform.Translation.GetX(), ViewportExtents.Min.GetY() + (ViewportExtents.Max.GetY() -  (Line.Start.GetY() * Transform.Scale.GetY() + Transform.Translation.GetY()))};
        OutLine.End  = {Line.End.GetX() * Transform.Scale.GetX() + Transform.Translation.GetX(), ViewportExtents.Min.GetY() + (ViewportExtents.Max.GetY() - (Line.End.GetY() * Transform.Scale.GetY() + Transform.Translation.GetY()))};    
    }

    void ToViewport(const ViewportTransform& Transform, const Range2D& ViewportExtents, const Algebra::Vector2D& Point, Algebra::Vector2D& OutPoint)
    {
        OutPoint.SetX( Point.GetX() * Transform.Scale.GetX() + Transform.Translation.GetX() );
        OutPoint.SetY( ViewportExtents.Min.GetY() + (ViewportExtents.Max.GetY() - (Point.GetY() * Transform.Scale.GetY() + Transform.Translation.GetY())) );    
    }

    void FromViewport(const ViewportTransform& Transform, const Range2D& ViewportExtents, const Algebra::Vector2D& Point, Algebra::Vector2D& OutPoint)
    {
        OutPoint.SetX( (Point.GetX() - Transform.Translation.GetX()) / Transform.Scale.GetX() );
        OutPoint.SetY( ((ViewportExtents.Min.GetY() + ViewportExtents.Max.GetY()) - Point.GetY() - Transform.Translation.GetY())/Transform.Scale.GetY() ); 
    }
    
    void DrawPlot(PlotPtr InPlot, const Range2D& ViewportWindow)
    {
        PlotBuffer.emplace_back(InPlot, ViewportWindow);
        MaximalDataRange |= InPlot->GetExtents();
    }

    void DrawLine(const Line2D& Line, ColorRGB Color)
    {
        LineBuffer.push_back({Line,Color});
    }

    void DrawText(const std::string& Text, const Algebra::Vector2D& Position, ColorRGB Color)
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

    PlotPtr ViewportPointInPlot(const Algebra::Vector2D& ViewportPosition, ViewportPointInPlotDelegateType&& ViewportPointInPlotDelegate)
    {
        if (GetPlotRange().IsEmpty())
        {
            return {};
        }
        for (const auto & Plot : PlotBuffer)
        {
            ViewportTransform Transform;
            GenerateTransform(GetPlotRange(), Plot.second, Transform);
            Algebra::Vector2D Position;
            FromViewport(Transform, Plot.second, ViewportPosition, Position);
            if ( Plot.first->GetExtents().IsPointInside(Position) )
            {
                std::pair<Algebra::Vector2D,Curve2D::MetaDataTagType> ClosestPointOnCurve = Plot.first->GetNearestPoint(Position);
                if ( Position.GetX() != 0.0f && Position.GetY()!=0.0f )
                {
                    ViewportPointInPlotDelegate(ClosestPointOnCurve.first, ClosestPointOnCurve.second);
                }
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
                    std::vector<Algebra::Vector2D> TransformedPoints;
                    ToViewport(Transform, ViewportWindowExtents, Curve.Points, TransformedPoints);

                    RendererImpl->DrawLine(
                                           TransformedPoints[0].GetX(),
                                           TransformedPoints[0].GetY(),
                                           TransformedPoints[1].GetX(),
                                           TransformedPoints[1].GetY(),
                                           Curve.Color);
                    RenderFilledCircle(TransformedPoints[0].GetX(), TransformedPoints[0].GetY(), 2.0f, Curve.Color);
                    RenderFilledCircle(TransformedPoints[1].GetX(), TransformedPoints[1].GetY(), 2.0f, Curve.Color);
                    
                    std::vector<Algebra::Vector2D> SampledPoints;
                    for (size_t n = 1; n < TransformedPoints.size()-2; ++n)
                    {
                        Curves::CatmullRomSegment2D SampleCurve(TransformedPoints[n-1], TransformedPoints[n], TransformedPoints[n+1], TransformedPoints[n+2]);
                        SampleCurve.SampleAdaptively(SampledPoints, 0.0f, 1.0f, 0.10f);
                        for (size_t nQ = 0; nQ < SampledPoints.size(); nQ+=2)
                        {
                            RendererImpl->DrawLine(
                                           SampledPoints[nQ+0].GetX(),
                                           SampledPoints[nQ+0].GetY(),
                                           SampledPoints[nQ+1].GetX(),
                                           SampledPoints[nQ+1].GetY(),
                                           Curve.Color);
                            RenderFilledCircle(SampledPoints[nQ+1].GetX(), SampledPoints[nQ+1].GetY(), 2.0f, Curve.Color);
                        }
                        SampledPoints.clear();
                    }
                }
                
                for (const auto & Line : Plot.first->Lines)
                {
                    Line2D TransformedLine;
                    ToViewport(Transform, ViewportWindowExtents, Line.first, TransformedLine);
                    RendererImpl->DrawLine(
                        TransformedLine.Start.GetX(),
                        TransformedLine.Start.GetY(),
                        TransformedLine.End.GetX(),
                        TransformedLine.End.GetY(),
                        Line.second);
                }

                for (const auto & Label : Plot.first->Labels)
                {
                    Algebra::Vector2D TransformedLabelPosition;
                    ToViewport(Transform,ViewportWindowExtents, Label.first.Position, TransformedLabelPosition);
                    RendererImpl->DrawText(Label.first.String, TransformedLabelPosition, Label.second);
                }

                for (const auto & Label : Plot.first->TransientLabels)
                {
                    Algebra::Vector2D TransformedLabelPosition;
                    ToViewport(Transform,ViewportWindowExtents, Label.first.Position, TransformedLabelPosition);
                    RendererImpl->DrawText(Label.first.String, TransformedLabelPosition, Label.second);
                }
                Plot.first->TransientLabels.clear();
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
            RendererImpl->DrawLine(Line.first.Start.GetX(), Line.first.Start.GetY(), Line.first.End.GetX(), Line.first.End.GetY(), Line.second);
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
