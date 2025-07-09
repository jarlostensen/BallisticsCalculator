
#include "Application.h"
#include "Plotter.h"
#include <vector>

#include <sstream>
//#include <SDL3/SDL_main.h>
#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>

using namespace Plotter;
namespace Application
{
    OnMouseMoveDelegateType OnMouseMoveDelegate;
    OnAppUpdateDelegateType OnAppUpdateDelegate;

    void SetMouseMoveDelegate(OnMouseMoveDelegateType&& InOnMouseMoveDelegate)
    {
        OnMouseMoveDelegate = std::move(InOnMouseMoveDelegate);
    }

    void SetAppUpdateDelegate(OnAppUpdateDelegateType&& InOnAppUpdateDelegate)
    {
        OnAppUpdateDelegate = std::move(InOnAppUpdateDelegate);
    }
    
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

    bool Init()
    {
        if ( !SDL_Init(SDL_INIT_VIDEO) )
        {
            SDL_Log("SDL could not initialize! SDL Error: %s\n", SDL_GetError());
            return false;
        }
        
        if (!SDL_CreateWindowAndRenderer("Ballistics Calculator", 
            static_cast<int>(ViewportExtents.Width()), static_cast<int>(ViewportExtents.Height()), 
            SDL_WINDOW_RESIZABLE, &SdlWindow, &SdlRenderer)) {
            SDL_Log("Couldn't create window and renderer: %s", SDL_GetError());
            return false;
        }

        if (!TTF_Init()) {
            SDL_Log("SDL_ttf could not initialize! SDL_ttf Error: %s\n", SDL_GetError());
            return false;
        }

        SDL_SetRenderDrawColor(SdlRenderer, 255, 255, 255, 255);
        SDL_RenderClear(SdlRenderer);

        if ( (SdlFont = TTF_OpenFont(R"(C:\Windows\Fonts\Arial.ttf)", 12))==nullptr )
        {
            SDL_Log("Failed to load font: SDL_Ttf error: %s\n", SDL_GetError());
            return false;
        }

        SetRenderer(std::make_shared<SdlRendererImpl>());
        int ViewportWidth;
        int ViewportHeight;
        SDL_GetRenderOutputSize(SdlRenderer, &ViewportWidth, &ViewportHeight);
        ViewportExtents.Max.SetX( static_cast<float>(ViewportWidth) );
        ViewportExtents.Max.SetY( static_cast<float>(ViewportHeight) );
        
        return true;
    }

    // SDL_AppResult SdlAppEvent(void* /*appstate*/, SDL_Event* event)
    // {
    //     if (event->type == SDL_EVENT_KEY_DOWN ||
    //     event->type == SDL_EVENT_QUIT) {
    //         return SDL_APP_SUCCESS;
    //     }
    //     switch (event->type)
    //     {
    //     case  SDL_EVENT_WINDOW_RESIZED:
    //         {
    //             int ViewportWidth;
    //             int ViewportHeight;
    //             SDL_GetRenderOutputSize(SdlRenderer, &ViewportWidth, &ViewportHeight);
    //             ViewportExtents.Max.SetX( static_cast<float>(ViewportWidth) );
    //             ViewportExtents.Max.SetY( static_cast<float>(ViewportHeight) );
    //         }
    //         break;
    //     case SDL_EVENT_MOUSE_BUTTON_DOWN:
    //     case SDL_EVENT_MOUSE_BUTTON_UP:
    //     case SDL_EVENT_MOUSE_MOTION:
    //         {
    //             OnMouseMoveDelegate({event->motion.x,event->motion.y});
    //         }
    //         break;
    //     default:;
    //     }
    //     return SDL_APP_CONTINUE;
    // }

    void Run()
    {
        bool bRunning = true;
        while (bRunning)
        {
            SDL_Event Event;
            while (SDL_PollEvent(&Event))
            {
                switch (Event.type)
                {
                case SDL_EVENT_QUIT:
                    bRunning = false;
                    break;
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
                        OnMouseMoveDelegate({Event.motion.x,Event.motion.y});
                    }
                    break;
                default:;
                }
                
                SDL_SetRenderDrawColor(SdlRenderer, 255, 255, 255, 255);
                SDL_RenderClear(SdlRenderer);
    
                SDL_SetRenderDrawColor(SdlRenderer, 64, 64, 64, 255);
                BeginFrame();
                if (Application::OnAppUpdateDelegate)
                {
                    Application::OnAppUpdateDelegate();
                }
                RenderFrame();
                EndFrame();
    
                SDL_RenderPresent(SdlRenderer);
            }
        }
    }
}