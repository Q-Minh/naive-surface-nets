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

	isosurface::regular_grid_t grid{};

	// contain the unit circle in a unit cube
	grid.x = -1.f;
	grid.y = -1.f;
	grid.z = -1.f;
	grid.sx = 100;
	grid.sy = 100;
	grid.sz = 100;
	grid.dx = (1.f - grid.x) / static_cast<float>(grid.sx);
	grid.dy = (1.f - grid.y) / static_cast<float>(grid.sy);
	grid.dz = (1.f - grid.z) / static_cast<float>(grid.sz);

	// add an outer layer to the unit cube so that the 
	// unit circle is contained in the interior of the
	// regular grid
	grid.x -= grid.dx;
	grid.y -= grid.dx;
	grid.z -= grid.dx;
	grid.sx += 2;
	grid.sy += 2;
	grid.sz += 2;

	common::igl_triangle_mesh mesh = isosurface::surface_nets(unit_circle, grid);

	igl::opengl::glfw::Viewer viewer{};
	viewer.data().set_mesh(mesh.V, mesh.F);
	viewer.launch();

	return 0;
}