#include "mesh.h"
#include "regular_grid.h"

#include <functional>

namespace isosurface {

common::igl_triangle_mesh surface_nets(
	std::function<float(float x, float y, float z)> const& implicit_function,
	regular_grid_t const& grid,
	float const isovalue = 0.f
);

} // isosurface