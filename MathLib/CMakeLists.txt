set(PROJECT_NAME MathLib)

add_library(MathLib INTERFACE
    include/Algebra.h
    include/Curves.h
)

target_include_directories(MathLib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
