cmake_minimum_required(VERSION 3.14)
project(naive-surface-nets)

add_executable(App)
set_target_properties(App PROPERTIES CXX_STANDARD 14)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

find_package(Eigen3 CONFIG REQUIRED)

get_target_property(eigen_include_directories Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
include_directories(SYSTEM ${eigen_include_directories})

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

include_directories(SYSTEM 
    thirdparty/
    thirdparty/libigl/include/
)
target_include_directories(App
PRIVATE
	include/
)

target_compile_features(App PRIVATE cxx_std_17)
target_compile_options(App PUBLIC /bigobj)