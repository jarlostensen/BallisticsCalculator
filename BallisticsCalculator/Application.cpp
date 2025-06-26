
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

    void PlotLine(float x0, float y0, float x1, float y1)
    {
        LineBuffer.push_back({
            {x0,y0}, {x1,y1}
        });
    }

    void ClearLines()
    {
        LineBuffer.clear();   
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

    void PlotText(const std::string& Text, const PointType2d& Position)
    {
        TextBuffer.push_back({
            Text,
            Position
        });
    }

    void DrawGrid()
    {
        if (MaximalDataRange.IsNonEmpty())
        {
            ClearLines();

            const float DataRangeMidY = MaximalDataRange.Min.y + MaximalDataRange.Height()/2;
            PlotLine(MaximalDataRange.Min.x, DataRangeMidY, MaximalDataRange.Max.x, DataRangeMidY);
            
            // try to fit one tick per 25 meters
            constexpr float TickSpacing = 25.0f;
            const int NumTicks = static_cast<int>(MaximalDataRange.Width() / TickSpacing);
            float X = MaximalDataRange.Min.x;
            for (int nTick = 0; nTick < NumTicks; nTick++)
            {
                constexpr float TickHalfHeight = 0.01f; //< one centimeter high
                const float ThisTickHalfHeight = (nTick & 1) ? TickHalfHeight : TickHalfHeight/2.0f; 
                PlotLine(X, DataRangeMidY - ThisTickHalfHeight, X, DataRangeMidY + ThisTickHalfHeight);
                X += TickSpacing;
            }
        }
    }
}

using namespace Ui;
namespace
{
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
        PointType2d ScaleTransform;
        PointType2d NormalizationTransform;
        GenerateTransforms(MaximalDataRange, ScaleTransform, NormalizationTransform);
        for (auto& Line : LineBuffer)
        {
            LineType2d TransformedLine;
            ViewportTransform(ScaleTransform, NormalizationTransform, Line, TransformedLine);
            SDL_RenderLine(SdlRenderer,
                           TransformedLine.first.x,
                           TransformedLine.first.y,
                           TransformedLine.second.x,
                           TransformedLine.second.y);
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

        if ( (SdlFont = TTF_OpenFont("C:\\Windows\\Fonts\\Arial.ttf", 12))==nullptr )
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

    DrawGrid();
    RenderText();
    
    SDL_RenderPresent(SdlRenderer);
    
    return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* /*appstate*/, SDL_AppResult /*result*/)
{
}