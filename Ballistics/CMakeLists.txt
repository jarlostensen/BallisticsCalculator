set(PROJECT_NAME Ballistics)

add_library(Ballistics
    include/Ballistics.h
    include/BulletData.h
    include/Data.h
    source/Ballistics.cpp
    source/BulletData.cpp
    source/Data.cpp
)

target_include_directories(Ballistics
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(Ballistics
    PUBLIC
        MathLib
)
