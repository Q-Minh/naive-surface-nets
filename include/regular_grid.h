#pragma once

namespace isosurface {

struct regular_grid_t
{
	// origin of the grid
	float x = 0.f, y = 0.f, z = 0.f;
	// voxel size in x, y, z
	float dx = 0.f, dy = 0.f, dz = 0.f;
	// number of voxels in x, y, z
	std::size_t sx = 0, sy = 0, sz = 0;
};

} // isosurface