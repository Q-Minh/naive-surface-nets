#include <chrono>
#include <igl/opengl/glfw/Viewer.h>
#include <mesh.h>
#include <surface_nets.h>

int main(int argc, char** argv)
{
    auto const unit_circle = [](float x, float y, float z) -> float {
        auto const len = std::sqrt(x * x + y * y + z * z);
        return len - 1.f;
    };

    auto const get_padded_voxel_grid = [](isosurface::point_t const& min,
                                          isosurface::point_t const& max,
                                          std::size_t dimensions[3]) -> isosurface::regular_grid_t {
        isosurface::regular_grid_t grid{};

        // contain the unit circle in a unit cube
        grid.x  = min.x;
        grid.y  = min.y;
        grid.z  = min.z;
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

    isosurface::point_t min{-1.f, -1.f, -1.f};
    isosurface::point_t max{1.f, 1.f, 1.f};
    std::size_t dimensions[3] = {300, 300, 300};
    auto const grid           = get_padded_voxel_grid(min, max, dimensions);

    std::cout << "Surface Nets grid dimensions: (" << dimensions[0] << ", " << dimensions[1] << ", "
              << dimensions[2] << ")\n";

    auto start                     = std::chrono::high_resolution_clock::now();
    common::igl_triangle_mesh mesh = isosurface::surface_nets(unit_circle, grid);
    auto end                       = std::chrono::high_resolution_clock::now();
    std::cout << "Surface Nets with no hint took: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << " ms\n";

    start = std::chrono::high_resolution_clock::now();
    mesh  = isosurface::par_surface_nets(unit_circle, grid);
    end   = std::chrono::high_resolution_clock::now();
    std::cout << "Parallel surface Nets took: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << " ms\n";

    start = std::chrono::high_resolution_clock::now();
    mesh  = isosurface::surface_nets(unit_circle, grid, isosurface::point_t{0.f, 0.f, .99f});
    end   = std::chrono::high_resolution_clock::now();
    std::cout << "Surface Nets with relatively good hint and parallel triangulation took: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << " ms\n";

    igl::opengl::glfw::Viewer viewer{};
    viewer.data().set_mesh(mesh.V, mesh.F);
    viewer.launch();

    return 0;
}