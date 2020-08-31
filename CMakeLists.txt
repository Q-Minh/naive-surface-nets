cmake_minimum_required(VERSION 3.14)
project(isosurface-extraction)

add_executable(App)
set_target_properties(App PROPERTIES CXX_STANDARD 14)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

find_package(Eigen3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
target_link_libraries(App PRIVATE glfw glad::glad Eigen3::Eigen)

set_target_properties(App PROPERTIES FOLDER Application)

target_sources(App 
PRIVATE
# headers
	include/mesh.h
    include/point.h
    include/regular_grid.h
	include/shared_vertex_mesh.h
    include/surface_nets.h

# sources
    src/surface_nets.cpp
    main.cpp
)

target_include_directories(App
PRIVATE
	include/
    thirdparty/
    thirdparty/libigl/include/
)

target_compile_options(App PUBLIC /bigobj)