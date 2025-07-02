
#include "Application.h"
#include <vector>
#define SDL_MAIN_USE_CALLBACKS 1  /* use the callbacks instead of main() */
#include <SDL3/SDL_main.h>
#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>

using namespace Ui;
namespace
{
    SDL_Window* SdlWindow = NULL;
    SDL_Renderer* SdlRenderer = NULL;
    TTF_Font* SdlFont = nullptr;
    Range2D ViewportExtents = {Point2D(0.0f,0.0f), Point2D(1200.0f,900.0f)};

    SDL_Texture* RenderText(const std::string &text, SDL_Color color) {
        SDL_Surface* textSurface = TTF_RenderText_Blended(SdlFont, text.data(), text.length(), color);
        SDL_Texture* textTexture = SDL_CreateTextureFromSurface(SdlRenderer, textSurface);
        SDL_DestroySurface(textSurface);
        return textTexture;
    }

    struct SdlRendererImpl : Ui::IRenderer
    {
        SdlRendererImpl() =  default;
        ~SdlRendererImpl() override = default;
        
        void DrawText(const std::string& Text, const Ui::Point2D& Position) override
        {
            SDL_Color TextColor = {0, 0, 0, 255};
            if ( SDL_Texture* TextTexture = RenderText(Text, TextColor) )
            {
                float textWidth, textHeight;
                SDL_GetTextureSize(TextTexture, &textWidth, &textHeight);
                SDL_FRect destRect = {Position.x, Position.y, textWidth, textHeight};
                SDL_RenderTexture(SdlRenderer, TextTexture, nullptr, &destRect);
                SDL_DestroyTexture(TextTexture);
            }
        }
        
        void DrawLine(float x0, float y0, float x1, float y1) override
        {
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

        SetRenderer(std::make_shared<SdlRendererImpl>());
        int ViewportWidth;
        int ViewportHeight;
        SDL_GetRenderOutputSize(SdlRenderer, &ViewportWidth, &ViewportHeight);
        ViewportExtents.Max.x = static_cast<float>(ViewportWidth);
        ViewportExtents.Max.y = static_cast<float>(ViewportHeight);
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
            int ViewportWidth;
            int ViewportHeight;
            SDL_GetRenderOutputSize(SdlRenderer, &ViewportWidth, &ViewportHeight);
            ViewportExtents.Max.x = static_cast<float>(ViewportWidth);
            ViewportExtents.Max.y = static_cast<float>(ViewportHeight);
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