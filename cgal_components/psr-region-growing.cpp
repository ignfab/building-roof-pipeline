#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/property_map.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing_on_point_set.h>
#include <CGAL/Polygonal_surface_reconstruction.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/polygon_mesh_to_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>

#ifdef CGAL_USE_SCIP // defined (or not) by CMake scripts, do not define by hand

#include <CGAL/SCIP_mixed_integer_program_traits.h>
typedef CGAL::SCIP_mixed_integer_program_traits<double> MIP_Solver;

#elif defined(CGAL_USE_GLPK) // defined (or not) by CMake scripts, do not define by hand

#include <CGAL/GLPK_mixed_integer_program_traits.h>
typedef CGAL::GLPK_mixed_integer_program_traits<double> MIP_Solver;

#endif

#if defined(CGAL_USE_GLPK) || defined(CGAL_USE_SCIP)

#include <fstream>
#include <CGAL/Timer.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;

typedef Kernel::FT FT;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;

// Point with normal, and plane index.
typedef boost::tuple<Point, Vector, int> PNI;
typedef std::vector<PNI> Point_vector;

typedef CGAL::Nth_of_tuple_property_map<0, PNI> Point_map;
typedef CGAL::Nth_of_tuple_property_map<1, PNI> Normal_map;
typedef CGAL::Nth_of_tuple_property_map<2, PNI> Plane_index_map;

typedef CGAL::Shape_detection::Point_set::K_neighbor_query<Kernel, Point_vector, Point_map> Neighbor_query;
typedef CGAL::Shape_detection::Point_set::Least_squares_plane_fit_region<Kernel, Point_vector, Point_map, Normal_map> Region_type;
typedef CGAL::Shape_detection::Region_growing<Point_vector, Neighbor_query, Region_type> Region_growing;

typedef CGAL::Surface_mesh<Point> Surface_mesh;
typedef CGAL::Polygonal_surface_reconstruction<Kernel> Polygonal_surface_reconstruction;
namespace PMP = CGAL::Polygon_mesh_processing;

class Index_map
{

public:
    using key_type = std::size_t;
    using value_type = int;
    using reference = value_type;
    using category = boost::readable_property_map_tag;

    Index_map() {}
    template <typename PointRange>
    Index_map(
        const PointRange &points,
        const std::vector<std::vector<std::size_t>> &regions) : m_indices(new std::vector<int>(points.size(), -1))
    {
        for (std::size_t i = 0; i < regions.size(); ++i)
            for (const std::size_t idx : regions[i])
                (*m_indices)[idx] = static_cast<int>(i);
    }

    inline friend value_type get(
        const Index_map &index_map,
        const key_type key)
    {
        const auto &indices = *(index_map.m_indices);
        return indices[key];
    }

private:
    std::shared_ptr<std::vector<int>> m_indices;
};

/*
* This example first extracts planes from the input point cloud
* (using region growing) and then reconstructs
* the surface model from the planes.
*/

int main(int argc, char *argv[])
{
    std::cout << "=> CGAL PSR Region Growing example started" << std::endl;

    Point_vector points;

    // Load point set from a file.
    const char* input_file = (argc > 1) ? argv[1] : "results/dsm_as_point_cloud_with_walls.ply";
    const std::string output_filename = (argc > 2) ? argv[2] :  "results/psr_region_growing.ply";
    const char* output_filename_faces = (argc > 3) ? argv[3] :  "results/psr_region_growing_candidate_faces.ply";

    std::ifstream input_stream(input_file);
    if (input_stream.fail())
    {
        std::cerr << "Failed open file \'" << input_file << "\'" << std::endl;
        return EXIT_FAILURE;
    }
    input_stream.close();
    std::cout << "Loading point cloud: " << input_file << "..." << std::endl;

    CGAL::Timer t;
    t.start();
    if (!CGAL::IO::read_points(input_file, std::back_inserter(points),
                               CGAL::parameters::point_map(Point_map()).normal_map(Normal_map())))
    {
        std::cerr << "Error: cannot read file " << input_file << std::endl;
        return EXIT_FAILURE;
    }
    else
        std::cout << "Done input points loading. " << points.size() << " points. Time: "
                  << t.time() << " sec." << std::endl;

    // PSR variables
    const double wt_fitting = (argc>4) ? std::atof(argv[4]) : 0.43;
    const double wt_coverage = (argc>5) ? std::atof(argv[5]) : 0.27;
    const double wt_complexity = (argc>6) ? std::atof(argv[6]) : 0.3;

    // Region Growing variables
    // CGAL default is 1
    FT max_distance_to_plane = (argc > 7) ? FT(std::stof(argv[7])) : FT(1);
    // CGAL default is 25
    FT max_accepted_angle = (argc > 8) ? FT(std::stof(argv[8])) : FT(25);
    // CGAL default is 3
    std::size_t min_region_size = (argc > 9) ? ((points.size() * std::stoi(argv[9])) / 100) : 3;
    // CGAL default value is 12 for nearest neighbour K
    std::size_t k = (argc > 10) ? std::stoi(argv[10]) : 10;

    // Create instances of the classes Neighbor_query and Region_type.
    Neighbor_query neighbor_query(points, k);
    Region_type region_type(
        points,
        max_distance_to_plane, max_accepted_angle, min_region_size);

    // Create an instance of the region growing class.
    Region_growing region_growing(
        points, neighbor_query, region_type);

    std::cout << "Extracting planes..." << std::endl;
    std::vector<std::vector<std::size_t>> regions;
    t.reset();
    region_growing.detect(std::back_inserter(regions));
    std::cout << "Done planes extraction. " << regions.size() << " planes extracted. Time: "
              << t.time() << " sec." << std::endl;

    // Stores the plane index of each point as the third element of the tuple.
    Index_map index_map(points, regions);
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        // Uses the get function from the property map that accesses the 3rd element of the tuple.
        const int plane_index = get(index_map, i);
        points[i].get<2>() = plane_index;
    }

    // Reconstruction.
    std::cout << "Generating candidate faces..." << std::endl;
    t.reset();
    Polygonal_surface_reconstruction algo(
        points,
        Point_map(),
        Normal_map(),
        Plane_index_map());
    std::cout << "Done generating candidate faces. Time: " << t.time() << " sec." << std::endl;

    Surface_mesh model;

    std::cout << "Reconstructing..." << std::endl;
    t.reset();
    if (!algo.reconstruct<MIP_Solver>(model,wt_fitting,wt_coverage,wt_complexity))
    {
        std::cerr << "Failed: " << algo.error_message() << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Done reconstructing mesh. Time: " << t.time() << " sec." << std::endl;

    // Cleanup and fix obtained mesh (holes, orientation, ...)
    t.reset();
    std::vector<Point> _points;
    std::vector<std::vector<std::size_t>> _polygons;
    Surface_mesh mesh;
    PMP::polygon_mesh_to_polygon_soup(model, _points, _polygons);
    PMP::repair_polygon_soup(_points, _polygons);
    PMP::orient_polygon_soup(_points, _polygons);
    PMP::polygon_soup_to_polygon_mesh(_points, _polygons, mesh);
    PMP::triangulate_faces(mesh);
    if (CGAL::is_closed(mesh))
    {
        std::cout << "The obtained mesh is closed" << std::endl;
        PMP::orient_to_bound_a_volume(mesh);
    }
    std::cout << "Done mesh cleanup. Time: " << t.time() << " sec." << std::endl;

    std::cout << "Saving..." << std::endl;
    t.reset();
    if (CGAL::IO::write_PLY(output_filename, mesh))
        std::cout << "Done writing result mesh. Saved to " << output_filename << ". Time: " << t.time() << " sec." << std::endl;
    else
    {
        std::cerr << "Failed saving file." << std::endl;
        return EXIT_FAILURE;
    }

    // Also stores the candidate faces as a surface mesh to a file
    Surface_mesh candidate_faces;
    algo.output_candidate_faces(candidate_faces);

    std::ofstream candidate_stream(output_filename_faces);
    if (CGAL::IO::write_PLY(candidate_stream, candidate_faces))
        std::cout << "Candidate faces saved to " << output_filename_faces << "." << std::endl;

    std::cout << "=> CGAL PSR Region Growing example finished\n" << std::endl;

    return EXIT_SUCCESS;
}

#else

int main(int, char **)
{
    std::cerr << "This test requires either GLPK or SCIP.\n";
    return EXIT_SUCCESS;
}

#endif // defined(CGAL_USE_GLPK) || defined(CGAL_USE_SCIP)