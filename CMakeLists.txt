cmake_minimum_required(VERSION 3.14)
project(naive-surface-nets VERSION 0.0.1 LANGUAGES CXX)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

include(FetchContent)

set(LIBIGL_USE_STATIC_LIBRARY     ON  CACHE STRING   "Use libigl as static library" )
set(LIBIGL_WITH_CGAL              OFF CACHE INTERNAL "Use CGAL"                     )
set(LIBIGL_WITH_COMISO            OFF CACHE INTERNAL "Use CoMiso"                   )
set(LIBIGL_WITH_CORK              OFF CACHE INTERNAL "Use Cork"                     )
set(LIBIGL_WITH_EMBREE            OFF CACHE INTERNAL "Use Embree"                   )
set(LIBIGL_WITH_MATLAB            OFF CACHE INTERNAL "Use Matlab"                   )
set(LIBIGL_WITH_MOSEK             OFF CACHE INTERNAL "Use MOSEK"                    )
set(LIBIGL_WITH_PNG               OFF CACHE INTERNAL "Use PNG"                      )
set(LIBIGL_WITH_TETGEN            OFF CACHE INTERNAL "Use Tetgen"                   )
set(LIBIGL_WITH_TRIANGLE          OFF CACHE INTERNAL "Use Triangle"                 )
set(LIBIGL_WITH_PREDICATES        OFF CACHE INTERNAL "Use exact predicates"         )
set(LIBIGL_WITH_XML               OFF CACHE INTERNAL "Use XML"                      )
set(LIBIGL_WITHOUT_COPYLEFT       OFF CACHE INTERNAL "Disable Copyleft libraries"   )
set(LIBIGL_WITH_OPENGL            ON  CACHE INTERNAL "download libigl with opengl"  )
set(LIBIGL_WITH_OPENGL_GLFW       ON  CACHE INTERNAL "download libigl with glfw"    )
set(LIBIGL_WITH_OPENGL_GLFW_IMGUI ON  CACHE INTERNAL "download libigl with imgui"   )
FetchContent_Declare(
  _libigl
  GIT_REPOSITORY https://github.com/libigl/libigl
  GIT_TAG        v2.2.0
)
FetchContent_MakeAvailable(_libigl)

add_executable(App)
set_target_properties(App PROPERTIES FOLDER naive-surface-nets)
target_compile_features(App PRIVATE cxx_std_17)
# target_compile_options(App PUBLIC /bigobj)

target_include_directories(App
PRIVATE
	include/
)

target_sources(App 
PRIVATE
# headers
	${CMAKE_CURRENT_SOURCE_DIR}/include/mesh.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/point.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/regular_grid.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/shared_vertex_mesh.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/surface_nets.h

# sources
    ${CMAKE_CURRENT_SOURCE_DIR}/src/surface_nets.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

target_link_libraries(App 
PRIVATE 
    igl::core 
    igl::opengl_glfw_imgui
)