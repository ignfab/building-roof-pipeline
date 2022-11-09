// STL includes.
#include <string>
#include <vector>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>

// CGAL includes.
#include <CGAL/memory.h>
#include <CGAL/IO/Color.h>
#include <CGAL/Iterator_range.h>
#include <CGAL/HalfedgeDS_vector.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing_on_polygon_mesh.h>

// Type declarations.
using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using FT = typename Kernel::FT;
using Point_3 = typename Kernel::Point_3;
using Color = CGAL::IO::Color;

// Choose the type of a container for a polygon mesh.
using Polygon_mesh = CGAL::Surface_mesh<Point_3>;
using Face_range = typename Polygon_mesh::Face_range;
using Neighbor_query = CGAL::Shape_detection::Polygon_mesh::One_ring_neighbor_query<Polygon_mesh>;
using Region_type = CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_region<Kernel, Polygon_mesh>;
using Sorting = CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_sorting<Kernel, Polygon_mesh, Neighbor_query>;
using Region = std::vector<std::size_t>;
using Regions = std::vector<Region>;
using Vertex_to_point_map = typename Region_type::Vertex_to_point_map;
using Region_growing = CGAL::Shape_detection::Region_growing<Face_range, Neighbor_query, Region_type, typename Sorting::Seed_map>;
using Face_index = typename Polygon_mesh::Face_index;
using Vertex_index = typename Polygon_mesh::Vertex_index;


int main(int argc, char *argv[])
{

    std::cout << "=> CGAL Region Growing on Mesh example started" << std::endl;

    Polygon_mesh polygon_mesh;

    // Read input mesh
    const char *filename = (argc > 1) ? argv[1] : "results/dsm_as_mesh.ply";
    std::ifstream in(filename);
    CGAL::IO::set_ascii_mode(in);
    if (!in || !CGAL::IO::read_PLY(in, polygon_mesh))
    {
        std::cerr << "Failed to read input mesh: " << filename << std::endl;
        return EXIT_FAILURE;
    }
    in.close();

    // Read mesh faces
    const Face_range face_range = faces(polygon_mesh);
    std::cout << "Loaded polygon mesh with "
              << face_range.size() << " faces"
              << std::endl;
    
    // Region Growing variables
    // CGAL default is 1
    FT max_distance_to_plane = (argc > 3) ? FT(std::stof(argv[3])) :  FT(1);
    // CGAL default is 25°
    FT max_accepted_angle = (argc > 4) ? FT(std::stof(argv[4])) :  FT(25);
    // CGAL default is 1 face
    std::size_t min_region_size = (argc > 5) ? (int)((face_range.size()*std::stoi(argv[5]))/100) : 1;

    // Create instances of the classes Neighbor_query and Region_type.
    Neighbor_query neighbor_query(polygon_mesh);

    const Vertex_to_point_map vertex_to_point_map(
        get(CGAL::vertex_point, polygon_mesh));

    Region_type region_type(
        polygon_mesh,
        max_distance_to_plane, max_accepted_angle, min_region_size,
        vertex_to_point_map);

    // Sort face indices.
    Sorting sorting(
        polygon_mesh, neighbor_query,
        vertex_to_point_map);
    sorting.sort();

    // Create an instance of the region growing class.
    Region_growing region_growing(
        face_range, neighbor_query, region_type,
        sorting.seed_map());

    // Run the algorithm.
    Regions regions;
    region_growing.detect(std::back_inserter(regions));

    // Print the number of found regions.
    std::cout << regions.size() << " regions have been found"
              << std::endl;

    // Save the result to a file in the user-provided path if any.
    srand(static_cast<unsigned int>(time(nullptr)));

    if (argc > 2)
    {
        bool created;

        // face color version
        // typename Polygon_mesh::template Property_map<Face_index, Color> face_color;
        // boost::tie(face_color, created) = polygon_mesh.template add_property_map<Face_index, Color>("f:color", Color(0, 0, 0));
      
        // vertex color version
        typename Polygon_mesh::template Property_map<Vertex_index, Color> vertex_color;
        boost::tie(vertex_color, created) = polygon_mesh.template add_property_map<Vertex_index, Color>("v:color", Color(0, 0, 0));
       
        if (!created)
        {
            std::cout << "region_growing_on_polygon_mesh example finished"
                      << std::endl;
            return EXIT_FAILURE;
        }

        const std::string fullpath = argv[2];
        std::ofstream out(fullpath);

        // Iterate through all regions.
        for (const auto &region : regions)
        {
            // Generate a random color.
            const Color color(
                static_cast<unsigned char>(rand() % 256),
                static_cast<unsigned char>(rand() % 256),
                static_cast<unsigned char>(rand() % 256));

            // Iterate through all region items.
            using size_type = typename Polygon_mesh::size_type;

            for (const auto index : region)
            {
                // face color version
                // Face_index faceindex = Face_index(static_cast<size_type>(index));

                // vertex color version
                Face_index fi = Face_index(static_cast<size_type>(index));
                Polygon_mesh::Halfedge_index hf = polygon_mesh.halfedge(fi);
                for(Polygon_mesh::Halfedge_index hi : halfedges_around_face(hf, polygon_mesh))
                {
                    Polygon_mesh::Vertex_index vi = target(hi, polygon_mesh);
                    vertex_color[vi] = color;
                }
            }
        }

        CGAL::IO::write_PLY(out, polygon_mesh);
        out.close();
        std::cout << "Resulting polygon mesh has been saved in "
                  << fullpath << std::endl;
    }

    std::cout << "=> CGAL Region Growing on Mesh example finished\n"
              << std::endl;

    return EXIT_SUCCESS;
}
