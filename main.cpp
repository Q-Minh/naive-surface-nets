#include <chrono>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <mesh.h>
#include <sstream>
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

    igl::opengl::glfw::Viewer viewer{};
    igl::opengl::glfw::imgui::ImGuiMenu menu{};
    viewer.plugins.push_back(&menu);
    menu.callback_draw_viewer_window = [&]() {
        static std::string sn_str;
        static std::string sn_parallel_str;
        static std::string sn_hint_str;

        static isosurface::point_t min{-1.f, -1.f, -1.f};
        static isosurface::point_t max{1.f, 1.f, 1.f};
        static std::size_t dimensions[3] = {300, 300, 300};

        ImGui::SetNextWindowSize(ImVec2(320, 640.f));
        ImGui::Begin("Naive Surface Nets");
        if (ImGui::CollapsingHeader("Isosurface Extraction", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (ImGui::TreeNode("Regular grid"))
            {
                static float float_step      = 0.1f;
                static float float_step_fast = 1.f;
                ImGui::Text("Min grid point");
                ImGui::InputFloat("x", &min.x, float_step, float_step_fast, "%.3f");
                ImGui::InputFloat("y", &min.y, float_step, float_step_fast, "%.3f");
                ImGui::InputFloat("z", &min.z, float_step, float_step_fast, "%.3f");

                ImGui::Text("Max grid point");
                ImGui::InputFloat("x", &max.x, float_step, float_step_fast, "%.3f");
                ImGui::InputFloat("y", &max.y, float_step, float_step_fast, "%.3f");
                ImGui::InputFloat("z", &max.z, float_step, float_step_fast, "%.3f");

                ImGui::Text("Dimensions");
                static std::size_t int_step      = 1u;
                static std::size_t int_step_fast = 10u;
                ImGui::InputScalar(
                    "sx",
                    ImGuiDataType_U64,
                    &dimensions[0],
                    &int_step,
                    &int_step_fast,
                    "%d");
                ImGui::InputScalar(
                    "sy",
                    ImGuiDataType_U64,
                    &dimensions[1],
                    &int_step,
                    &int_step_fast,
                    "%d");
                ImGui::InputScalar(
                    "sz",
                    ImGuiDataType_U64,
                    &dimensions[2],
                    &int_step,
                    &int_step_fast,
                    "%d");

                ImGui::TreePop();
            }

            if (ImGui::Button("Extract sphere"))
            {
                auto const grid = get_padded_voxel_grid(min, max, dimensions);

                std::ostringstream oss{};

                auto start                     = std::chrono::high_resolution_clock::now();
                common::igl_triangle_mesh mesh = isosurface::surface_nets(unit_circle, grid);
                auto end                       = std::chrono::high_resolution_clock::now();
                oss << "Surface Nets with no hint took:\n"
                    << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
                    << " ms\n";

                sn_str = oss.str();
                oss.flush();

                start = std::chrono::high_resolution_clock::now();
                mesh  = isosurface::par_surface_nets(unit_circle, grid);
                end   = std::chrono::high_resolution_clock::now();
                oss << "Parallel surface Nets took:\n"
                    << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
                    << " ms\n";

                sn_parallel_str = oss.str();
                oss.flush();

                start = std::chrono::high_resolution_clock::now();
                mesh  = isosurface::surface_nets(
                    unit_circle,
                    grid,
                    isosurface::point_t{0.f, 0.f, .99f});
                end = std::chrono::high_resolution_clock::now();
                oss << "Surface Nets with relatively good hint\nand parallel triangulation took:\n"
                    << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
                    << " ms\n";

                sn_hint_str = oss.str();
                oss.flush();

                viewer.data().clear();
                viewer.data().set_mesh(mesh.V, mesh.F);
            }

            ImGui::BulletText(sn_str.c_str());
            ImGui::BulletText(sn_parallel_str.c_str());
            ImGui::BulletText(sn_hint_str.c_str());
        }
        if (ImGui::CollapsingHeader("Visualization", ImGuiTreeNodeFlags_DefaultOpen))
        {
            ImGui::Checkbox(
                "Wireframe",
                [&]() { return static_cast<bool>(viewer.data().show_lines); },
                [&](bool value) { viewer.data().show_lines = value; });
        }
        ImGui::End();
    };
    viewer.launch();

    return 0;
}