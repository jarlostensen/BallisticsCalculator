cmake_minimum_required(VERSION 3.16.0 FATAL_ERROR)
set(CMAKE_SYSTEM_VERSION 10.0 CACHE STRING "" FORCE)
project(BallisticsCalculator LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Option for SDL support
option(WITH_SDL "Build with SDL support" ON)

if(WITH_SDL)
    add_compile_definitions(WITH_SDL=1)
    message("Building with SDL support")
endif()

# Add subdirectories for each component
add_subdirectory(MathLib)
add_subdirectory(Ballistics)
add_subdirectory(BallisticsCalculator)
add_subdirectory(Tests)
add_subdirectory(UiLib)

if(WITH_SDL)
    add_subdirectory(ThirdParty/SDL)
endif()
