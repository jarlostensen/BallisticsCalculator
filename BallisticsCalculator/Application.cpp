
#include "Application.h"
#include <vector>
#define SDL_MAIN_USE_CALLBACKS 1  /* use the callbacks instead of main() */
#include <SDL3/SDL_main.h>
#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>

namespace
{
    SDL_Window* SdlWindow = NULL;
    SDL_Renderer* SdlRenderer = NULL;
    TTF_Font* SdlFont = nullptr;    
}

namespace Ui
{
    using LineBufferType = std::vector<Line2D>;
    using CurveBufferType = std::vector<Curve2D>;
    using TextBufferType = std::vector<Label2D>;
    using PlotBufferType = std::vector<PlotPtr>;
    LineBufferType LineBuffer;
    CurveBufferType CurveBuffer;
    TextBufferType TextBuffer;
    PlotBufferType PlotBuffer;

    struct ViewportTransform
    {
        Point2D Scale;
        Point2D Translation;
    };

    constexpr Range2D ViewportMargins{{2.0f,2.0f}, {2.0f,2.0f}};
    Range2D ViewportExtents = {Point2D(0.0f,0.0f) + ViewportMargins.Min, Point2D(1200.0f,900.0f) - ViewportMargins.Max};
    Range2D MaximalDataRange = EmptyRange2D;

    void UpdateViewportExtents()
    {
        int ViewPortWidth;
        int ViewPortHeight;
        SDL_GetRenderOutputSize(SdlRenderer, &ViewPortWidth, &ViewPortHeight);
        ViewportExtents.Min.x = 0.0f;
        ViewportExtents.Min.y = 0.0f;
        ViewportExtents.Max.x = static_cast<float>(ViewPortWidth);
        ViewportExtents.Max.y = static_cast<float>(ViewPortHeight);
    }

    void GenerateTransform(const Range2D& Extents, const Range2D& InViewportExtents, ViewportTransform& OutViewportTransform)
    {
        OutViewportTransform.Scale.x = (InViewportExtents.Max.x - InViewportExtents.Min.x) / ( Extents.Max.x -  Extents.Min.x);
        OutViewportTransform.Scale.y = (InViewportExtents.Max.y - InViewportExtents.Min.y) / ( Extents.Max.y -  Extents.Min.y);
        OutViewportTransform.Translation.x = ( Extents.Max.x * InViewportExtents.Min.x -  Extents.Min.x * InViewportExtents.Max.x) / ( Extents.Max.x -  Extents.Min.x);
        OutViewportTransform.Translation.y = ( Extents.Max.y * InViewportExtents.Min.y -  Extents.Min.y * InViewportExtents.Max.y) / ( Extents.Max.y -  Extents.Min.y);
    }

    void GenerateTransform(const Range2D& Extents, ViewportTransform& OutViewportTransform)
    {
        OutViewportTransform.Scale.x = (ViewportExtents.Max.x - ViewportExtents.Min.x) / ( Extents.Max.x -  Extents.Min.x);
        OutViewportTransform.Scale.y = (ViewportExtents.Max.y - ViewportExtents.Min.y) / ( Extents.Max.y -  Extents.Min.y);
        OutViewportTransform.Translation.x = ( Extents.Max.x * ViewportExtents.Min.x -  Extents.Min.x * ViewportExtents.Max.x) / ( Extents.Max.x -  Extents.Min.x);
        OutViewportTransform.Translation.y = ( Extents.Max.y * ViewportExtents.Min.y -  Extents.Min.y * ViewportExtents.Max.y) / ( Extents.Max.y -  Extents.Min.y);
    }
    
    void ToViewport(const ViewportTransform& Transform, const std::vector<Point2D>& Points, std::vector<Point2D>& OutPoints)
    {
        OutPoints.resize(Points.size());
        for (size_t n = 0; n < Points.size(); ++n)
        {
            OutPoints[n].x = Points[n].x * Transform.Scale.x + Transform.Translation.x;
            OutPoints[n].y = ViewportExtents.Max.y - (Points[n].y * Transform.Scale.y + Transform.Translation.y);
        }
    }

    void ToViewport(const ViewportTransform& Transform, const Line2D& Line, Line2D& OutLine)
    {
        OutLine.Start = {Line.Start.x * Transform.Scale.x + Transform.Translation.x, ViewportExtents.Max.y -  (Line.Start.y * Transform.Scale.y + Transform.Translation.y)};
        OutLine.End  = {Line.End.x * Transform.Scale.x + Transform.Translation.x, ViewportExtents.Max.y - (Line.End.y * Transform.Scale.y + Transform.Translation.y)};    
    }

    void ToViewport(const ViewportTransform& Transform, const Point2D& Point, Point2D& OutPoint)
    {
        OutPoint.x = Point.x * Transform.Scale.x + Transform.Translation.x;
        OutPoint.y = ViewportExtents.Max.y - (Point.y * Transform.Scale.y + Transform.Translation.y);    
    }
    
    void DrawLine(float x0, float y0, float x1, float y1)
    {
        LineBuffer.push_back({
            {x0,y0}, {x1,y1}
        });
    }

    void AddPlot(PlotPtr InPlot)
    {
        PlotBuffer.push_back(InPlot);
    }

    void DrawText(const std::string& Text, const Point2D& Position)
    {
        TextBuffer.push_back({
            Text,
            Position
        });
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
            SDL_RenderLine(SdlRenderer, 
                centerX - x, centerY + y, 
                centerX + x, centerY + y);
            SDL_RenderLine(SdlRenderer, 
                centerX - x, centerY - y, 
                centerX + x, centerY - y);
            SDL_RenderLine(SdlRenderer, 
                centerX - y, centerY + x, 
                centerX + y, centerY + x);
            SDL_RenderLine(SdlRenderer, 
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

    static void RenderLines()
    {
        if (LineBuffer.empty())
        {
            return;
        }
        for (auto& Line : LineBuffer)
        {
            SDL_RenderLine(SdlRenderer,
                           Line.Start.x,
                           Line.Start.y,
                           Line.End.x,
                           Line.End.y);
        }
    }

    SDL_Texture* RenderText(const std::string &text, SDL_Color color) {
        SDL_Surface* textSurface = TTF_RenderText_Blended(SdlFont, text.data(), text.length(), color);
        SDL_Texture* textTexture = SDL_CreateTextureFromSurface(SdlRenderer, textSurface);
        SDL_DestroySurface(textSurface);
        return textTexture;
    }

    static void RenderText()
    {
        if ( SdlFont == nullptr )
        {
            return;
        }
        for (auto& Text : TextBuffer)
        {
            SDL_Color TextColor = {0, 0, 0, 255};
            if ( SDL_Texture* TextTexture = RenderText(Text.String, TextColor) )
            {
                float textWidth, textHeight;
                SDL_GetTextureSize(TextTexture, &textWidth, &textHeight);
                SDL_FRect destRect = {Text.Position.x, Text.Position.y, textWidth, textHeight};
                SDL_RenderTexture(SdlRenderer, TextTexture, nullptr, &destRect);
                SDL_DestroyTexture(TextTexture);
            }
        }
    }
    
    SDL_AppResult SdlInit(void** appstate, int argc, char* argv[])
    {
        (void)appstate;
        (void)argc;
        (void)argv;
        if (!SDL_CreateWindowAndRenderer("Ballistics Calculator", 800, 600, SDL_WINDOW_RESIZABLE, &SdlWindow, &SdlRenderer)) {
            SDL_Log("Couldn't create window and renderer: %s", SDL_GetError());
            return SDL_APP_FAILURE;
        }

        if (!TTF_Init()) {
            SDL_Log("SDL_ttf could not initialize! SDL_ttf Error: %s\n", SDL_GetError());
            return SDL_APP_FAILURE;
        }

        SDL_SetRenderDrawColor(SdlRenderer, 255, 255, 255, 255);
        SDL_RenderClear(SdlRenderer);

        if ( (SdlFont = TTF_OpenFont(R"(C:\Windows\Fonts\Arial.ttf)", 12))==nullptr )
        {
            SDL_Log("Failed to load font: SDL_Ttf error: %s\n", SDL_GetError());
        }
        
        
        AppInit();

        return SDL_APP_CONTINUE;
    }

    SDL_AppResult SdlAppEvent(void* /*appstate*/, SDL_Event* event)
    {
        if (event->type == SDL_EVENT_KEY_DOWN ||
        event->type == SDL_EVENT_QUIT) {
            return SDL_APP_SUCCESS;
        }
        if (event->type == SDL_EVENT_WINDOW_RESIZED)
        {
            UpdateViewportExtents();
            AppUpdate();
        }
        return SDL_APP_CONTINUE;
    }

    class PlotRenderer
    {
    public:
        static void RenderPlots(const PlotBufferType& Plots)
        {
            //TESTING:
            Range2D ViewportWindowExtents = ViewportExtents;
            ViewportWindowExtents.Max.y *= 0.75f;
            
            for (auto& Plot : Plots)
            {
                ViewportTransform Transform;
                GenerateTransform(Plot->GetExtents(), ViewportWindowExtents, Transform);
                for (const auto & Curve : Plot->Curves)
                {
                    std::vector<Point2D> TransformedPoints;
                    ToViewport(Transform, Curve.Points, TransformedPoints);
                    Point2D& PrevPoint = TransformedPoints[0];
                    for (size_t n = 1; n < TransformedPoints.size(); ++n)
                    {
                        SDL_RenderLine(SdlRenderer,
                                       PrevPoint.x,
                                       PrevPoint.y,
                                       TransformedPoints[n].x,
                                       TransformedPoints[n].y);
                        RenderFilledCircle(TransformedPoints[n].x, TransformedPoints[n].y, 2.0f);
                        PrevPoint = TransformedPoints[n];
                    }
                    RenderFilledCircle(PrevPoint.x, PrevPoint.y, 2.0f);
                }
                
                for (const auto & Line : Plot->Lines)
                {
                    Line2D TransformedLine;
                    ToViewport(Transform, Line, TransformedLine);
                    SDL_RenderLine(SdlRenderer,
                        TransformedLine.Start.x,
                        TransformedLine.Start.y,
                        TransformedLine.End.x,
                        TransformedLine.End.y);
                }

                for (const auto & Label : Plot->Labels)
                {
                    SDL_Color TextColor = {0, 0, 0, 255};
                    if ( SDL_Texture* TextTexture = RenderText(Label.String, TextColor) )
                    {
                        float textWidth, textHeight;
                        SDL_GetTextureSize(TextTexture, &textWidth, &textHeight);
                        Point2D TransformedLabelPosition;
                        ToViewport(Transform, Label.Position, TransformedLabelPosition);
                        SDL_FRect destRect = {TransformedLabelPosition.x, TransformedLabelPosition.y, textWidth, textHeight};
                        SDL_RenderTexture(SdlRenderer, TextTexture, nullptr, &destRect);
                        SDL_DestroyTexture(TextTexture);
                    }
                }
            }
        }
    };
}
using namespace Renderer;

SDL_AppResult SDL_AppInit(void** appstate, int argc, char* argv[])
{
    return SdlInit(appstate, argc, argv);
}

SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event)
{
    return SdlAppEvent(appstate, event);   
}

SDL_AppResult SDL_AppIterate(void* /*appstate*/)
{
    SDL_SetRenderDrawColor(SdlRenderer, 255, 255, 255, 255);
    SDL_RenderClear(SdlRenderer);
    
    SDL_SetRenderDrawColor(SdlRenderer, 64, 64, 64, 255);
    PlotRenderer::RenderPlots(PlotBuffer);
    
    SDL_SetRenderDrawColor(SdlRenderer, 128, 128, 128, 255);
    RenderLines();

    RenderText();
    
    SDL_RenderPresent(SdlRenderer);
    
    return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* /*appstate*/, SDL_AppResult /*result*/)
{
}