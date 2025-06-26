
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
    struct Text
    {
        std::string String;
        PointType2d Position;
    };

    using LineBufferType = std::vector<LineType2d>;
    using CurveBufferType = std::vector<Curve2D>;
    using TextBufferType = std::vector<Text>;
    LineBufferType LineBuffer;
    CurveBufferType CurveBuffer;
    TextBufferType TextBuffer;

    constexpr Range2D ViewportMargins{{2.0f,2.0f}, {2.0f,2.0f}};
    Range2D ViewportExtents = {Point2D(0.0f,0.0f) + ViewportMargins.Min, Point2D(800.0f,600.0f) - ViewportMargins.Max};
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

    void GenerateTransforms(const Range2D& CurveDataExtents, PointType2d& OutScaleTransform, PointType2d& OutNormalizationTransform)
    {
        OutScaleTransform.x = (ViewportExtents.Max.x - ViewportExtents.Min.x) / (CurveDataExtents.Max.x - CurveDataExtents.Min.x);
        OutScaleTransform.y = (ViewportExtents.Max.y - ViewportExtents.Min.y) / (CurveDataExtents.Max.y - CurveDataExtents.Min.y);
        OutNormalizationTransform.x = (CurveDataExtents.Max.x * ViewportExtents.Min.x - CurveDataExtents.Min.x * ViewportExtents.Max.x) / (CurveDataExtents.Max.x - CurveDataExtents.Min.x);
        OutNormalizationTransform.y = (CurveDataExtents.Max.y * ViewportExtents.Min.y - CurveDataExtents.Min.y * ViewportExtents.Max.y) / (CurveDataExtents.Max.y - CurveDataExtents.Min.y);
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

    void ViewportTransform(const PointType2d& ScaleTransform, const PointType2d& NormalizationTransform,
        const LineType2d& Line, LineType2d& OutLine)
    {
        OutLine.first = {Line.first.x * ScaleTransform.x + NormalizationTransform.x, ViewportExtents.Max.y -  (Line.first.y * ScaleTransform.y + NormalizationTransform.y)};
        OutLine.second = {Line.second.x * ScaleTransform.x + NormalizationTransform.x, ViewportExtents.Max.y - (Line.second.y * ScaleTransform.y + NormalizationTransform.y)};    
    }

    void DataPointToViewport(const PointType2d& ScaleTransform, const PointType2d& NormalizationTransform,
        const PointType2d& Point, PointType2d& OutPoint)
    {
        OutPoint.x = Point.x * ScaleTransform.x + NormalizationTransform.x;
        OutPoint.y = ViewportExtents.Max.y -  (Point.y * ScaleTransform.y + NormalizationTransform.y);
    }

    void GenerateTransforms(PointType2d& OutScaleTransform, PointType2d& OutNormalizationTransform)
    {
        GenerateTransforms(MaximalDataRange, OutScaleTransform, OutNormalizationTransform);
    }

    void DataVectorToViewport(const PointType2d& ScaleTransform, const PointType2d& NormalizationTransform, const PointType2d& Vector, PointType2d& OutVector)
    {
        DataPointToViewport(ScaleTransform, NormalizationTransform, {Vector.x + MaximalDataRange.Min.x, Vector.y + MaximalDataRange.Min.y}, OutVector);
    }

    void DrawLine(float x0, float y0, float x1, float y1)
    {
        LineBuffer.push_back({
            {x0,y0}, {x1,y1}
        });
    }

    void ClearLines()
    {
        LineBuffer.clear();   
    }

    void ClearText()
    {
        TextBuffer.clear();  
    }

    void PlotCurve(const Curve2D& Curve)
    {
        CurveBuffer.push_back(Curve);
        MaximalDataRange |= Curve.Extents;
    }

    void ClearCurves()
    {
        CurveBuffer.clear();
        MaximalDataRange = EmptyRange2D;
    }

    void DrawText(const std::string& Text, const PointType2d& Position)
    {
        TextBuffer.push_back({
            Text,
            Position
        });
    }

    Range2D GetPlotRange()
    {
        return MaximalDataRange;   
    }

   
}

using namespace Ui;
namespace
{
    void RenderFilledCircle(float centerX, float centerY, float radius)
    {
        // Using the midpoint circle algorithm
        const float diameter = radius * 2;
        float x = radius - 1;
        float y = 0;
        float dx = 1;
        float dy = 1;
        float error = dx - diameter;

        while (x >= y) {
            // Draw horizontal lines for each octant to fill the circle
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

    void RenderCurves()
    {
        for (auto& Curve : CurveBuffer)
        {
            std::vector<Point2D> TransformedPoints;
            PointType2d ScaleTransform;
            PointType2d NormalizationTransform;
            GenerateTransforms(MaximalDataRange, ScaleTransform, NormalizationTransform);
            ViewportTransform(ScaleTransform, NormalizationTransform, Curve.Points, TransformedPoints);

            Point2D& PrevPoint = TransformedPoints[0];
            for (size_t n = 1; n < TransformedPoints.size(); ++n)
            {
                SDL_FPoint Center = {TransformedPoints[n].x, TransformedPoints[n].y};
                RenderFilledCircle(TransformedPoints[n].x, TransformedPoints[n].y, 2.0f);

                SDL_RenderLine(SdlRenderer,
                               PrevPoint.x,
                               PrevPoint.y,
                               TransformedPoints[n].x,
                               TransformedPoints[n].y);
                PrevPoint = TransformedPoints[n];
            }
        }
    }

    void RenderLines()
    {
        if (LineBuffer.empty())
        {
            return;
        }
        for (auto& Line : LineBuffer)
        {
            SDL_RenderLine(SdlRenderer,
                           Line.first.x,
                           Line.first.y,
                           Line.second.x,
                           Line.second.y);
        }
    }

    SDL_Texture* RenderText(const std::string &text, SDL_Color color) {
        SDL_Surface* textSurface = TTF_RenderText_Blended(SdlFont, text.data(), text.length(), color);
        SDL_Texture* textTexture = SDL_CreateTextureFromSurface(SdlRenderer, textSurface);
        SDL_DestroySurface(textSurface);
        return textTexture;
    }

    void RenderText()
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
}

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
    RenderCurves();
    
    SDL_SetRenderDrawColor(SdlRenderer, 128, 128, 128, 255);
    RenderLines();

    RenderText();
    
    SDL_RenderPresent(SdlRenderer);
    
    return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* /*appstate*/, SDL_AppResult /*result*/)
{
}