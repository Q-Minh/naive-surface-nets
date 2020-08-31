#include "surface_nets.h"

#include <array>
#include <unordered_map>
#include <numeric>

namespace isosurface {

common::igl_triangle_mesh surface_nets(
	std::function<float(float x, float y, float z)> const& implicit_function,
	regular_grid_t const& grid,
	float const isovalue)
{
	isosurface::mesh mesh{};

	struct mesh_bounding_box_t
	{
		point_t min{ 0.f, }, max{ 0.f, };
	};

	// bounding box of the mesh in coordinate frame of the mesh
	mesh_bounding_box_t const mesh_bounding_box
	{
		{ grid.x                  , grid.y                  , grid.z },
		{ grid.x + grid.sx*grid.dx, grid.y + grid.sy*grid.dy, grid.z + grid.sz*grid.dz }
	};

	// mapping from 3d coordinates to 1d
	auto const get_active_cube_index = [](std::size_t x, std::size_t y, std::size_t z, regular_grid_t const& grid) -> std::size_t
	{
		return x + (y * grid.sx) + (z * grid.sx * grid.sy);
	};

	// mapping from active cube indices to vertex indices of the generated mesh
	std::unordered_map<std::size_t, std::uint64_t> active_cube_to_vertex_index_map{};

	/*
	* Vertex generation and placement
	* 
	* We visit every voxel of the voxel grid, that is every cube of the 
	* regular 3d grid, and determine which ones are intersected by the
	* implicit surface defined by the implicit function. To do so, we 
	* look for bipolar edges. Bipolar edges are edges for which their 
	* vertices (v1,v2) have associated scalar values for which either:
	* 
	* f(v1) >= isovalue and f(v2) < isovalue
	* or 
	* f(v1) < isovalue and f(v2) >= isovalue
	* 
	* is true, where f is the implicit function.
	* 
	* Walking over every voxel, we look for bipolar edges, and if we 
	* find at least one, the voxel is marked as an active cube. Every 
	* active cube must generate one vertex of the resulting mesh that
	* resides in that cube. Computing the position of the generated vertex
	* in the active cube is what we call vertex placement.
	*/
	for (std::size_t k = 0; k < grid.sz; ++k)
	for (std::size_t j = 0; j < grid.sy; ++j)
	for (std::size_t i = 0; i < grid.sx; ++i)
	{
		/*
		* 
		* Visit every voxel in the regular grid with the following configuration
		* 
		* Coordinate frame
		*     
		*       z y
		*       |/
		*       o--x
		* 
		* Voxel corner indices
		* 
		*        7          6
		*        o----------o 
		*       /|         /|
		*     4/ |       5/ |
		*     o--|-------o  |
		*     |  o-------|--o
		*     | /3       | /2
		*     |/         |/
		*     o----------o
		*     0          1
		* 
		*/

		// coordinates of voxel corners in voxel grid coordinate frame
		point_t const voxel_corner_grid_positions[8] =
		{
			{ static_cast<float>(i    ), static_cast<float>(j    ), static_cast<float>(k    ) },
			{ static_cast<float>(i + 1), static_cast<float>(j    ), static_cast<float>(k    ) },
			{ static_cast<float>(i + 1), static_cast<float>(j + 1), static_cast<float>(k    ) },
			{ static_cast<float>(i    ), static_cast<float>(j + 1), static_cast<float>(k    ) },
			{ static_cast<float>(i    ), static_cast<float>(j    ), static_cast<float>(k + 1) },
			{ static_cast<float>(i + 1), static_cast<float>(j    ), static_cast<float>(k + 1) },
			{ static_cast<float>(i + 1), static_cast<float>(j + 1), static_cast<float>(k + 1) },
			{ static_cast<float>(i    ), static_cast<float>(j + 1), static_cast<float>(k + 1) },
		};

		// coordinates of voxel corners in the mesh's coordinate frame
		point_t const voxel_corner_positions[8] =
		{
			{ grid.x + i     * grid.dx, grid.y + j     * grid.dy, grid.z + k     * grid.dz },
			{ grid.x + (i+1) * grid.dx, grid.y + j     * grid.dy, grid.z + k     * grid.dz },
			{ grid.x + (i+1) * grid.dx, grid.y + (j+1) * grid.dy, grid.z + k     * grid.dz },
			{ grid.x + i     * grid.dx, grid.y + (j+1) * grid.dy, grid.z + k     * grid.dz },
			{ grid.x + i     * grid.dx, grid.y + j     * grid.dy, grid.z + (k+1) * grid.dz },
			{ grid.x + (i+1) * grid.dx, grid.y + j     * grid.dy, grid.z + (k+1) * grid.dz },
			{ grid.x + (i+1) * grid.dx, grid.y + (j+1) * grid.dy, grid.z + (k+1) * grid.dz },
			{ grid.x + i     * grid.dx, grid.y + (j+1) * grid.dy, grid.z + (k+1) * grid.dz }
		};

		// scalar values of the implicit function evaluated at cube vertices (voxel corners)
		float const voxel_corner_values[8] =
		{
			implicit_function(voxel_corner_positions[0].x, voxel_corner_positions[0].y, voxel_corner_positions[0].z),
			implicit_function(voxel_corner_positions[1].x, voxel_corner_positions[1].y, voxel_corner_positions[1].z),
			implicit_function(voxel_corner_positions[2].x, voxel_corner_positions[2].y, voxel_corner_positions[2].z),
			implicit_function(voxel_corner_positions[3].x, voxel_corner_positions[3].y, voxel_corner_positions[3].z),
			implicit_function(voxel_corner_positions[4].x, voxel_corner_positions[4].y, voxel_corner_positions[4].z),
			implicit_function(voxel_corner_positions[5].x, voxel_corner_positions[5].y, voxel_corner_positions[5].z),
			implicit_function(voxel_corner_positions[6].x, voxel_corner_positions[6].y, voxel_corner_positions[6].z),
			implicit_function(voxel_corner_positions[7].x, voxel_corner_positions[7].y, voxel_corner_positions[7].z)
		};

		// the edges provide indices to the corresponding current cube's vertices (voxel corners)
		std::size_t const edges[12][2] =
		{
			{ 0u, 1u },
			{ 1u, 2u },
			{ 2u, 3u },
			{ 3u, 0u },
			{ 4u, 5u },
			{ 5u, 6u },
			{ 6u, 7u },
			{ 7u, 4u },
			{ 0u, 4u },
			{ 1u, 5u },
			{ 2u, 6u },
			{ 3u, 7u }
		};

		auto const is_scalar_positive = [](float scalar, float isovalue) -> bool
		{
			return scalar >= isovalue;
		};

		auto const are_edge_scalars_bipolar = [&is_scalar_positive](float scalar1, float scalar2, float isovalue) -> bool
		{
			return is_scalar_positive(scalar1, isovalue) != is_scalar_positive(scalar2, isovalue);
		};

		bool const edge_bipolarity_array[12] =
		{
			are_edge_scalars_bipolar(voxel_corner_values[edges[0 ][0]], voxel_corner_values[edges[0 ][1]], isovalue),
			are_edge_scalars_bipolar(voxel_corner_values[edges[1 ][0]], voxel_corner_values[edges[1 ][1]], isovalue),
			are_edge_scalars_bipolar(voxel_corner_values[edges[2 ][0]], voxel_corner_values[edges[2 ][1]], isovalue),
			are_edge_scalars_bipolar(voxel_corner_values[edges[3 ][0]], voxel_corner_values[edges[3 ][1]], isovalue),
			are_edge_scalars_bipolar(voxel_corner_values[edges[4 ][0]], voxel_corner_values[edges[4 ][1]], isovalue),
			are_edge_scalars_bipolar(voxel_corner_values[edges[5 ][0]], voxel_corner_values[edges[5 ][1]], isovalue),
			are_edge_scalars_bipolar(voxel_corner_values[edges[6 ][0]], voxel_corner_values[edges[6 ][1]], isovalue),
			are_edge_scalars_bipolar(voxel_corner_values[edges[7 ][0]], voxel_corner_values[edges[7 ][1]], isovalue),
			are_edge_scalars_bipolar(voxel_corner_values[edges[8 ][0]], voxel_corner_values[edges[8 ][1]], isovalue),
			are_edge_scalars_bipolar(voxel_corner_values[edges[9 ][0]], voxel_corner_values[edges[9 ][1]], isovalue),
			are_edge_scalars_bipolar(voxel_corner_values[edges[10][0]], voxel_corner_values[edges[10][1]], isovalue),
			are_edge_scalars_bipolar(voxel_corner_values[edges[11][0]], voxel_corner_values[edges[11][1]], isovalue),
		};

		// an active voxel must have at least one bipolar edge
		bool const is_voxel_active = edge_bipolarity_array[0]  || 
			                         edge_bipolarity_array[1]  || 
			                         edge_bipolarity_array[2]  || 
			                         edge_bipolarity_array[3]  || 
			                         edge_bipolarity_array[4]  || 
			                         edge_bipolarity_array[5]  || 
			                         edge_bipolarity_array[6]  || 
			                         edge_bipolarity_array[7]  || 
			                         edge_bipolarity_array[8]  || 
			                         edge_bipolarity_array[9]  || 
			                         edge_bipolarity_array[10] || 
			                         edge_bipolarity_array[11];

		// cubes that are not active do not generate mesh vertices
		if (!is_voxel_active)
			continue;

		// store all edge intersection points with the implicit surface in voxel grid coordinates
		std::vector<point_t> edge_intersection_points;

		// visit every bipolar edge
		for (std::size_t e = 0; e < 12; ++e)
		{
			if (!edge_bipolarity_array[e])
				continue;

			// get points p1, p2 of the edge e in grid coordinates
			auto const p1 = voxel_corner_grid_positions[edges[e][0]];
			auto const p2 = voxel_corner_grid_positions[edges[e][1]];

			// get value of the implicit function at edge vertices
			auto const s1 = voxel_corner_values[edges[e][0]];
			auto const s2 = voxel_corner_values[edges[e][1]];

			// perform linear interpolation using implicit function
			// values at vertices
			auto const t = (isovalue - s1) / (s2 - s1);
			edge_intersection_points.emplace_back(
				p1 + t * (p2 - p1)
			);
		}

		/*
		* We approximate the generated mesh vertex using the geometric center of all 
		* bipolar edges' intersection point with the implicit surface. The geometric 
		* center if first computed in local voxel grid coordinates and will then be 
		* mapped to the mesh's coordinates later.
		*/
		float const number_of_intersection_points = static_cast<float>(edge_intersection_points.size());
		point_t const sum_of_intersection_points = std::accumulate(
			edge_intersection_points.cbegin(), 
			edge_intersection_points.cend(), 
			point_t{ 0.f, 0.f, 0.f }
		);
		point_t const geometric_center_of_edge_intersection_points = sum_of_intersection_points / number_of_intersection_points;

		/*
		* 
		* map geometric center from local grid coordinates to world coordinates
		* 
		* with local grid (lp1, lp2) and world grid (wp1, wp2), 
		* and a point lp in local grid mapped to wp in world grid, 
		* we have that:
		* 
		* (lp - lp1) / (lp2 - lp1) = (wp - wp1) / (wp2 - wp1),
		* Our local grid is the voxel grid with lp1=(0, 0, 0) and lp2=(sx, sy, sz), 
		* and our world grid is the mesh's bounding box, wp1=(minx, miny, minz) and wp2=(maxx, maxy, maxz),
		* and we have a local point lp="geometric center of edge intersection points" mapped to wp, 
		* which we are looking for, such that:
		* 
		* wp = wp1 + (wp2 - wp1) * (lp - lp1) / (lp2 - lp1)
		* 
		*/
		point_t const mesh_vertex =
		{
			mesh_bounding_box.min.x + 
				(mesh_bounding_box.max.x - mesh_bounding_box.min.x) * (geometric_center_of_edge_intersection_points.x - 0.f) / (static_cast<float>(grid.sx) - 0.f),
			mesh_bounding_box.min.y + 																							
				(mesh_bounding_box.max.y - mesh_bounding_box.min.y) * (geometric_center_of_edge_intersection_points.y - 0.f) / (static_cast<float>(grid.sy) - 0.f),
			mesh_bounding_box.min.z + 																							
				(mesh_bounding_box.max.z - mesh_bounding_box.min.z) * (geometric_center_of_edge_intersection_points.z - 0.f) / (static_cast<float>(grid.sz) - 0.f),
		};

		/*
		* Store mapping from this active cube to the mesh's vertex index 
		* for triangulation later on.
		*/
		std::size_t const active_cube_index = get_active_cube_index(i, j, k, grid);
		std::uint64_t const vertex_index = mesh.vertex_count();
		active_cube_to_vertex_index_map[active_cube_index] = vertex_index;

		mesh.add_vertex(mesh_vertex);
	}

	/*
	* Triangulation
	* 
	* For triangulation, we need not iterate over every voxel. We simply 
	* visit every active cube and look at neighbors with which there is 
	* a possible triangulation to be made. In the surface nets algorithm,
	* a quad is generated when four active cubes share a common edge. 
	* As such, at each iteration, we will look at neighboring voxels of 
	* the current active cube, and if they are all active too, we 
	* generate a quad and triangulate the quad.
	*/
	for (auto const& key_value : active_cube_to_vertex_index_map)
	{
		std::size_t   const active_cube_index = key_value.first;
		std::uint64_t const vertex_index      = key_value.second;

		/*
		* Knowing active_cube_index = i + j*sx + k*sx*sy, 
		* we can recover i,j,k using only active_cube_index and sx,sy using:
		* i = index % xmax
		* j = (index / xmax) % ymax
		* k = index / (xmax * ymax)
		*/
		std::size_t i = (active_cube_index)           % grid.sx;
		std::size_t j = (active_cube_index / grid.sx) % grid.sy;
		std::size_t k = (active_cube_index) / (grid.sx * grid.sy);

		auto const is_lower_boundary_cube = [](std::size_t i, std::size_t j, std::size_t k, regular_grid_t const& g)
		{
			return (i == 0 || j == 0 || k == 0);
		};

		/* 
		* we define a  lower boundary cube to be a cube with (i=0       || j=0       || k=0	    )
		* we define an upper boundary cube to be a cube with (i >= sx-1 || j >= sy-1 || k >=sz-1)
		* 
		* lower boundary cubes have missing neighbor voxels, so we don't triangulate
		* when the current voxel is a boundary cube. Our method of quad generation considers 
		* the following potentially crossed edges of an active cube:
		* 
		* Active cube
		* 
		*        7          6
		*        o----------o
		*       /|         /|
		*     4/ |       5/ |
		*     o--|-------o  |
		*     |  o-------|--o
		*     | /3       | /2
		*     |/         |/
		*     o----------o
		*     0          1
		*
		* Potentially crossed edges
		* 
		*     4
		*     o
		*     |  o
		*     | /3
		*     |/
		*     o----------o
		*     0          1
		* 
		* By considering only these potential crossed edges for each active cube, 
		* we make sure that every interior edge of the voxel grid is visited only 
		* once. Additional voxel grid edges are also visited at upper boundary cubes.
		* 
		* To make sure that these additional edge do not cause any problems, 
		* we should make sure that the implicit surface 
		* (level-set of the implicit function with level=isovalue) 
		* never crosses those additional edges. One way to do that is to add an 
		* outer layer to the voxel grid, that is to augment the grid dimensions 
		* and center the implicit function in the augmented grid. That way, we 
		* know for sure that the implicit surface only crossed interior edges of 
		* the augmented grid (since we know it used to fit the non-augmented grid).
		* Currently, we leave this to the user to give us an implicit function for which
		* the level-set of the given isovalue defines an isosurface contained entirely
		* in interior voxel grid cubes or to give us an augmented grid otherwise.
		* 
		*/
		if (is_lower_boundary_cube(i, j, k, grid))
			continue;

		/*
		* 
		* In coordinate frame:
		* 
		*     z y
		*     |/
		*     o--x
		* 
		* For potentially crossed edges, we generate quads connecting the 4 cubes of
		* the crossed edge, so the considered neighbors are:
		* 
		* 
		*           o----------o----------o
		* 		   /|		  /|	     /|
		*         / |   0    / |        / | <---- this is the current active cube (i,j,k) in the above coordinate frame
		*        o--|-------o--|-------o  |
		*       /|  o------/|--o------/|--o
		*      / | /|1    / | /|2    / | /|
		*     o--|/-|--5-o--|/-|--4-o  |/ |
		*     |  o--|----|--o--|----|--o  |
		*     | /|  o----|-/|--o----|-/|--o
		*     |/ | /     |/	| /3    |/ | /
		*     o--|/------o--|/------o  |/
		*        o-------|--o-------|--o
		* 	             | /        | /
		* 	             |/	        |/
		* 	             o----------o
		* 
		* Indices of neighbor voxels are written on the top squares of the cubes.
		* So upper cubes are 0, 1, 2 and lower cubes are 3, 4, 5. 
		*/
		std::size_t const neighbor_grid_positions[6][3] =
		{
			{ i - 1, j    , k     },
			{ i - 1, j - 1, k     },
			{ i    , j - 1, k     },
			{ i    , j - 1, k - 1 },
			{ i    , j    , k - 1 },
			{ i - 1, j    , k - 1 }
		};

		point_t const voxel_corners_of_interest[4] =
		{
			// vertex 0
			{ grid.x + i       * grid.dx, grid.y + j       * grid.dy, grid.z + k       * grid.dz },
			// vertex 4
			{ grid.x + i       * grid.dx, grid.y + j       * grid.dy, grid.z + (k + 1) * grid.dz },
			// vertex 3
			{ grid.x + i       * grid.dx, grid.y + (j + 1) * grid.dy, grid.z + k       * grid.dz },
			// vertex 1
			{ grid.x + (i + 1) * grid.dx, grid.y + j       * grid.dy, grid.z + k       * grid.dz }
		};

		float const edge_scalar_values[3][2] =
		{
			// directed edge (0,4)
			{ 
				implicit_function(voxel_corners_of_interest[0].x, voxel_corners_of_interest[0].y, voxel_corners_of_interest[0].z),
				implicit_function(voxel_corners_of_interest[1].x, voxel_corners_of_interest[1].y, voxel_corners_of_interest[1].z)
			},
			// directed edge (3,0)
			{
				implicit_function(voxel_corners_of_interest[2].x, voxel_corners_of_interest[2].y, voxel_corners_of_interest[2].z),
				implicit_function(voxel_corners_of_interest[0].x, voxel_corners_of_interest[0].y, voxel_corners_of_interest[0].z)
			},
			// directed edge (0,1)
			{
				implicit_function(voxel_corners_of_interest[0].x, voxel_corners_of_interest[0].y, voxel_corners_of_interest[0].z),
				implicit_function(voxel_corners_of_interest[3].x, voxel_corners_of_interest[3].y, voxel_corners_of_interest[3].z)
			}
		};

		/*
		* The three potentially generated quads all share the current
		* active cube's mesh vertex. Store the current active cube's 
		* neighbors for each potentially generated quad.
		* 
		* The order of the neighbors is such that the quads are generated 
		* with an outward normal direction that is in the same direction
		* as the directed/oriented edges:
		* (0, 4), (3, 0) and (0, 1)
		* 
		* Then, looking at the gradient of the implicit function along 
		* the edge, we can flip the quad's outward normal direction 
		* to match the gradient's direction.
		*/
		std::size_t const quad_neighbors[3][3] =
		{
			{ 0, 1, 2 },
			{ 0, 5, 4 },
			{ 2, 3, 4 }
		};

		/*
		* For directed edge e that has the same direction as the 
		* gradient along e, the correct order of neighbor vertices 
		* is 0,1,2. If the direction of e is opposite the gradient, 
		* the correct order is 2,1,0.
		* 
		* For current active cube's mesh vertex v, we will then have 
		* quads: v,0,1,2 or v,2,1,0
		*/
		std::array<std::size_t, 3> const quad_neighbor_orders[2] =
		{
			{ 0, 1, 2 },
			{ 2, 1, 0 }
		};

		// look at each potentially generated quad
		for (std::size_t i = 0; i < 3; ++i)
		{
			auto const neighbor1 = get_active_cube_index(
				neighbor_grid_positions[quad_neighbors[i][0]][0],
				neighbor_grid_positions[quad_neighbors[i][0]][1],
				neighbor_grid_positions[quad_neighbors[i][0]][2],
				grid);

			auto const neighbor2 = get_active_cube_index(
				neighbor_grid_positions[quad_neighbors[i][1]][0],
				neighbor_grid_positions[quad_neighbors[i][1]][1],
				neighbor_grid_positions[quad_neighbors[i][1]][2],
				grid);

			auto const neighbor3 = get_active_cube_index(
				neighbor_grid_positions[quad_neighbors[i][2]][0],
				neighbor_grid_positions[quad_neighbors[i][2]][1],
				neighbor_grid_positions[quad_neighbors[i][2]][2],
				grid);

			/*
			* Only generate a quad if all neighbors are active cubes.
			* If a neighbor is an active cube, our mapping should
			* contain it.
			*/
			if (active_cube_to_vertex_index_map.count(neighbor1) == 0 ||
				active_cube_to_vertex_index_map.count(neighbor2) == 0 ||
				active_cube_to_vertex_index_map.count(neighbor3) == 0)
				continue;

			std::size_t const neighbor_vertices[3] =
			{
				active_cube_to_vertex_index_map.at(neighbor1),
				active_cube_to_vertex_index_map.at(neighbor2),
				active_cube_to_vertex_index_map.at(neighbor3)
			};
			
			auto const& neighbor_vertices_order =
				edge_scalar_values[i][1] > edge_scalar_values[i][0] ?
				quad_neighbor_orders[0] :
				quad_neighbor_orders[1];

			/*
			* Generate the quad q = (v0,v1,v2,v3)
			*/
			auto const v0 = vertex_index;
			auto const v1 = neighbor_vertices[neighbor_vertices_order[0]];
			auto const v2 = neighbor_vertices[neighbor_vertices_order[1]];
			auto const v3 = neighbor_vertices[neighbor_vertices_order[2]];

			/*
			* Triangulate the quad q naively. 
			* 
			* To produce better quality triangulations, 
			* we could also triangulate based on the  
			* diagonal that minimizes the maximum triangle angle
			* to have better regularity in the mesh.
			* 
			* We can also verify that the 2 generated triangles are
			* in the envelope of the current bipolar edge and if 
			* not, generate 4 triangles instead of 2. The four 
			* triangles are formed by creating diagonals between 
			* all of the quad's corners (the mesh vertices) and 
			* the current edge's intersection point. By "current
			* edge", we mean the edge shared by all 3 neighbor
			* active cubes and the current active cube.
			*/
			mesh.add_face({ v0, v1, v2 });
			mesh.add_face({ v0, v2, v3 });
		}
	}

	return common::convert::from(mesh);
}

} // isosurface
