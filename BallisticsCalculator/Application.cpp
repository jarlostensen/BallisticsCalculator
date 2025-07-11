﻿
#include "Application.h"
#include <vector>
#define SDL_MAIN_USE_CALLBACKS 1  /* use the callbacks instead of main() */
#include <sstream>
#include <SDL3/SDL_main.h>
#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>

using namespace Plotter;
namespace
{
    SDL_Window* SdlWindow = NULL;
    SDL_Renderer* SdlRenderer = NULL;
    TTF_Font* SdlFont = nullptr;
    Range2D ViewportExtents = {Algebra::Vector2D(0.0f,0.0f), Algebra::Vector2D(1600.0f,900.0f)};

    SDL_Texture* RenderText(const std::string &text, SDL_Color color) {
        SDL_Surface* textSurface = TTF_RenderText_Blended(SdlFont, text.data(), text.length(), color);
        SDL_Texture* textTexture = SDL_CreateTextureFromSurface(SdlRenderer, textSurface);
        SDL_DestroySurface(textSurface);
        return textTexture;
    }

    struct SdlRendererImpl : Plotter::IRenderer
    {
        SdlRendererImpl() =  default;
        ~SdlRendererImpl() override = default;
        
        void DrawText(const std::string& Text, const Algebra::Vector2D& Position, ColorRGB Color) override
        {
            std::istringstream stream(Text);
            std::string ParsedLine;
            std::vector<std::string> Lines;
            while (std::getline(stream, ParsedLine)) {
                Lines.push_back(ParsedLine);
            }

            SDL_Color TextColor = {Color.R, Color.G, Color.B, 255};
            float LineOffset = 0.0f;
            for (const auto& Line : Lines)
            {
                if ( SDL_Surface* TextSurface = TTF_RenderText_Blended(SdlFont, Line.data(), Line.length(), TextColor) )
                {
                    if ( SDL_Texture* TextTexture = SDL_CreateTextureFromSurface(SdlRenderer, TextSurface) )
                    {
                        float TextWidth, TextHeight;
                        SDL_GetTextureSize(TextTexture, &TextWidth, &TextHeight);
                        SDL_FRect DestRect = {Position.GetX(), Position.GetY() + LineOffset, TextWidth, TextHeight};
                        SDL_RenderTexture(SdlRenderer, TextTexture, nullptr, &DestRect);
                        LineOffset += TextHeight;
                        SDL_DestroyTexture(TextTexture);           
                    }
                    SDL_DestroySurface(TextSurface);
                }
            }
        }
        
        void DrawLine(float x0, float y0, float x1, float y1, ColorRGB Color) override
        {
            SDL_SetRenderDrawColor(SdlRenderer, Color.R, Color.G, Color.B, 255);
            SDL_RenderLine(SdlRenderer, x0, y0, x1, y1);
        }
        
        Range2D GetViewportExtents() override
        {
            return ViewportExtents;
        }
    };

    SDL_AppResult SdlInit(void** appstate, int argc, char* argv[])
    {
        (void)appstate;
        (void)argc;
        (void)argv;
        if (!SDL_CreateWindowAndRenderer("Ballistics Calculator", 
            static_cast<int>(ViewportExtents.Width()), static_cast<int>(ViewportExtents.Height()), 
            SDL_WINDOW_RESIZABLE, &SdlWindow, &SdlRenderer)) {
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

        SetRenderer(std::make_shared<SdlRendererImpl>());
        int ViewportWidth;
        int ViewportHeight;
        SDL_GetRenderOutputSize(SdlRenderer, &ViewportWidth, &ViewportHeight);
        ViewportExtents.Max.SetX( static_cast<float>(ViewportWidth) );
        ViewportExtents.Max.SetY( static_cast<float>(ViewportHeight) );
        AppInit();

        return SDL_APP_CONTINUE;
    }

    SDL_AppResult SdlAppEvent(void* /*appstate*/, SDL_Event* event)
    {
        if (event->type == SDL_EVENT_KEY_DOWN ||
        event->type == SDL_EVENT_QUIT) {
            return SDL_APP_SUCCESS;
        }
        switch (event->type)
        {
        case  SDL_EVENT_WINDOW_RESIZED:
            {
                int ViewportWidth;
                int ViewportHeight;
                SDL_GetRenderOutputSize(SdlRenderer, &ViewportWidth, &ViewportHeight);
                ViewportExtents.Max.SetX( static_cast<float>(ViewportWidth) );
                ViewportExtents.Max.SetY( static_cast<float>(ViewportHeight) );
            }
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
        case SDL_EVENT_MOUSE_BUTTON_UP:
        case SDL_EVENT_MOUSE_MOTION:
            {
                PlotPtr Plot = ViewportPointInPlot({event->motion.x,event->motion.y}, [](const Algebra::Vector2D& Point, Curve2D::MetaDataTagType Tag)
                {
                    AppHitDelegate(Point, Tag);
                });
            }
            break;
        default:;
        }
        return SDL_APP_CONTINUE;
    }
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
    BeginFrame();
    AppUpdate();
    RenderFrame();
    EndFrame();
    
    //SDL_SetRenderDrawColor(SdlRenderer, 128, 128, 128, 255);
    //RenderLines();
    //RenderText();
    
    SDL_RenderPresent(SdlRenderer);
    
    return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void* /*appstate*/, SDL_AppResult /*result*/)
{
}
    