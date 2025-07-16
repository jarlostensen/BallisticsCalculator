set(PROJECT_NAME BallisticsCalculator)


add_executable(BallisticsCalculator
    Application.cpp
    Application.h
    main.cpp
)

set(SDL_INCLUDES "../ThirdParty/SDL/include")
set(SDL_TTF_INCLUDES "../ThirdParty/SDL3_ttf-3.2.2/include")
set(SDL_TTF_BINARY "${CMAKE_SOURCE_DIR}/ThirdParty/SDL3_ttf-3.2.2/lib/x64/SDL3_ttf.lib")

target_include_directories(BallisticsCalculator PRIVATE ${SDL_INCLUDES} ${SDL_TTF_INCLUDES})

target_link_libraries(BallisticsCalculator
    PUBLIC
        UiLib 
        Ballistics
        MathLib
)

if(WITH_SDL)
    target_link_libraries(BallisticsCalculator
        PRIVATE
            SDL3::SDL3
            ${SDL_TTF_BINARY}
    )
endif()
