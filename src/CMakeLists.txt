cmake_minimum_required(VERSION 2.8)

# Application source
set(APPLICATION_SOURCE

    # Static scene
    static_scene/light.cpp
    static_scene/circle.cpp
    static_scene/object.cpp

    sampler.cpp
    bbox.cpp
    bsdf.cpp
    svg.cpp
    png.cpp
    triangulation.cpp
    texture.cpp
    pathtracer.cpp
    character.cpp
    timeline.cpp
    hardware_renderer.cpp
    viewport.cpp
    main.cpp
)

#-------------------------------------------------------------------------------
# Set include directories
#-------------------------------------------------------------------------------
include_directories(
  ${GLEW_INCLUDE_DIRS}
  ${GLFW_INCLUDE_DIRS}
  ${CMU462_INCLUDE_DIRS}
  ${FREETYPE_INCLUDE_DIRS}
)

#-------------------------------------------------------------------------------
# Set link directories
#-------------------------------------------------------------------------------
link_directories(
  ${GLEW_LIBRARY_DIRS}
  ${GLFW_LIBRARY_DIRS}
  ${CMU462_LIBRARY_DIRS}
  ${FREETYPE_LIBRARY_DIRS}
)

#-------------------------------------------------------------------------------
# Add executable
#-------------------------------------------------------------------------------
add_executable(pathtracer ${APPLICATION_SOURCE})

target_link_libraries( pathtracer
    CMU462 ${CMU462_LIBRARIES}
    glew ${GLEW_LIBRARIES}
    glfw ${GLFW_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${FREETYPE_LIBRARIES}
    ${CMAKE_THREADS_INIT}
)

#-------------------------------------------------------------------------------
# Platform-specific configurations for target
#-------------------------------------------------------------------------------
if(APPLE)
  set_property( TARGET pathtracer APPEND_STRING PROPERTY COMPILE_FLAGS
                "-Wno-deprecated-declarations -Wno-c++11-extensions")
endif(APPLE)

# Put executable in build directory root
set(EXECUTABLE_OUTPUT_PATH ..)

# Install to project root
install(TARGETS pathtracer DESTINATION ${pathtracer_SOURCE_DIR})
