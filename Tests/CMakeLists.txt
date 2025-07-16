set(PROJECT_NAME Tests)

add_executable(Tests
    Tests.cpp
)

target_link_libraries(Tests
    PUBLIC
        UiLib
        Ballistics
)
