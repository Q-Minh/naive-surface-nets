#include <mesh.h>
#include <surface_nets.h>

#include <igl/opengl/glfw/Viewer.h>

int main(int argc, char** argv)
{
	auto const unit_circle = [](float x, float y, float z) -> float
	{
		auto const len = std::sqrt(x * x + y * y + z * z);
		return len - 1.f;
	};

	auto const get_padded_voxel_grid = [](
		isosurface::point_t const& min, 
		isosurface::point_t const& max, 
		std::size_t dimensions[3]) -> isosurface::regular_grid_t
	{
		isosurface::regular_grid_t grid{};

		// contain the unit circle in a unit cube
		grid.x = min.x;
		grid.y = min.y;
		grid.z = min.z;
		grid.sx = dimensions[0];
		grid.sy = dimensions[1];
		grid.sz = dimensions[2];
		grid.dx = (max.x - min.x) / static_cast<float>(grid.sx);
		grid.dy = (max.y - min.y) / static_cast<float>(grid.sy);
		grid.dz = (max.z - min.z) / static_cast<float>(grid.sz);

		// add an outer layer to the unit cube so that the 
		// unit circle is contained in the interior of the
		// regular grid
		grid.x -= grid.dx;
		grid.y -= grid.dx;
		grid.z -= grid.dx;
		grid.sx += 2;
		grid.sy += 2;
		grid.sz += 2;

		return grid;
	};

	isosurface::point_t min{ -1.f, -1.f, -1.f };
	isosurface::point_t max{ 1.f, 1.f, 1.f };
	std::size_t dimensions[3] = { 100, 100, 100 };
	auto const grid = get_padded_voxel_grid(
		min, 
		max, 
		dimensions
	);

	common::igl_triangle_mesh mesh = isosurface::surface_nets(unit_circle, grid);

	igl::opengl::glfw::Viewer viewer{};
	viewer.data().set_mesh(mesh.V, mesh.F);
	viewer.launch();

	return 0;
}