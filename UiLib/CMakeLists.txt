set(PROJECT_NAME UiLib)

add_library(UiLib
    include/Plotter.h
    source/Plotter.cpp
)

target_include_directories(UiLib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(UiLib
    PUBLIC
        MathLib
)
