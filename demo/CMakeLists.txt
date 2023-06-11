cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl -lpthread")

#add_subdirectory(libs/glfw)
file(GLOB IMGUI_FILES    
    "libs/imgui/*.cpp"
)

file(GLOB SRC_FILES
    "src/*.cpp"
)

add_executable(
    ${CMAKE_PROJECT_NAME} 
    main.cpp
    ${IMGUI_FILES}
    ${SRC_FILES}
)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 20)

find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)
find_library(GLFW NAMES libglfw3.a)

target_link_libraries(
    ${PROJECT_NAME} 
    PUBLIC 
    ${GLFW}
    ${OPENGL_gl_LIBRARY}
    engine
)

target_include_directories(
    ${PROJECT_NAME} 
    PUBLIC
    libs/imgui
    "include"
)