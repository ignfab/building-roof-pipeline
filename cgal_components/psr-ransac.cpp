// A simple Polyfit with Efficient RANSAC example to understand its inner working
// Adapted from https://doc.cgal.org/latest/Polygonal_surface_reconstruction/Polygonal_surface_reconstruction_2polyfit_example_without_input_planes_8cpp-example.html

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/property_map.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
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

#include <CGAL/Timer.h>

#include <fstream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;

// Point with normal, and plane index
typedef boost::tuple<Point, Vector, int> PNI;
typedef std::vector<PNI> Point_vector;
typedef CGAL::Nth_of_tuple_property_map<0, PNI> Point_map;
typedef CGAL::Nth_of_tuple_property_map<1, PNI> Normal_map;
typedef CGAL::Nth_of_tuple_property_map<2, PNI> Plane_index_map;

typedef CGAL::Shape_detection::Efficient_RANSAC_traits<Kernel, Point_vector, Point_map, Normal_map> Traits;
typedef CGAL::Shape_detection::Efficient_RANSAC<Traits> Efficient_ransac;
typedef CGAL::Shape_detection::Plane<Traits> Plane;
typedef CGAL::Shape_detection::Point_to_shape_index_map<Traits> Point_to_shape_index_map;
typedef CGAL::Polygonal_surface_reconstruction<Kernel> Polygonal_surface_reconstruction;
typedef CGAL::Surface_mesh<Point> Surface_mesh;

typedef Surface_mesh::Face_index Face_index;
namespace PMP = CGAL::Polygon_mesh_processing;

// This visitor is copied from https://doc.cgal.org/latest/Polygon_mesh_processing/Polygon_mesh_processing_2orient_polygon_soup_example_8cpp-example.html
// Optional visitor for orientating a polygon soup to demonstrate usage for some functions.
// inherits from the default class as some functions are not overloaded
struct Visitor : public PMP::Default_orientation_visitor
{
    void non_manifold_edge(std::size_t id1, std::size_t id2, std::size_t nb_poly)
    {
        std::cout << "The edge " << id1 << ", " << id2 << " is not manifold: " << nb_poly << " incident polygons." << std::endl;
    }
    void non_manifold_vertex(std::size_t id, std::size_t nb_cycles)
    {
        std::cout << "The vertex " << id << " is not manifold: " << nb_cycles << " connected components of vertices in the link." << std::endl;
    }
    void duplicated_vertex(std::size_t v1, std::size_t v2)
    {
        std::cout << "The vertex " << v1 << " has been duplicated, its new id is " << v2 << "." << std::endl;
    }
    void vertex_id_in_polygon_replaced(std::size_t p_id, std::size_t i1, std::size_t i2)
    {
        std::cout << "In the polygon " << p_id << ", the index " << i1 << " has been replaced by " << i2 << "." << std::endl;
    }
    void polygon_orientation_reversed(std::size_t p_id)
    {
        std::cout << "The polygon " << p_id << " has been reversed." << std::endl;
    }
};

/*
 * This example first extracts planes from the input point cloud
 * (using Efficient RANSAC) and then reconstructs
 * the surface model from the planes.
 */
int main(int argc, char *argv[])
{

    std::cout << "=> CGAL PSR RANSAC example started" << std::endl;

    // Objects
    Point_vector points;
    CGAL::Timer t;

    // Input output variables
    const char *input_file = (argc > 1) ? argv[1] : "results/dsm_as_point_cloud_with_walls.ply";
    const char *output_file = (argc > 2) ? argv[2] : "results/psr_ransac.ply";
    const char *candidate_faces_file = (argc > 3) ? argv[3] : "results/psr_ransac_candidate_faces.ply";

    // Open input point cloud file
    std::ifstream input_stream(input_file);
    if (input_stream.fail())
    {
        std::cerr << "Failed to open file \'" << input_file << "\'" << std::endl;
        return EXIT_FAILURE;
    }
    input_stream.close();

    // Load input point cloud file
    std::cout << "Loading point cloud: " << input_file << "..."<< std::endl;

    t.start();
    if (!CGAL::IO::read_points(input_file, std::back_inserter(points),
                               CGAL::parameters::point_map(Point_map()).normal_map(Normal_map())))
    {
        std::cerr << "Error: cannot read file " << input_file << std::endl;
        return EXIT_FAILURE;
    }
    else
        std::cout << "Done input points loading. " << points.size() << " points loaded. Time: " << t.time() << " sec." << std::endl;

    // Polyfit variables
    const double wt_fitting = (argc > 4) ? std::atof(argv[4]) : 0.43;
    const double wt_coverage = (argc > 5) ? std::atof(argv[5]) : 0.27;
    const double wt_complexity = (argc > 6) ? std::atof(argv[6]) : 0.3;

    // Efficient RANSAC variables
    Efficient_ransac::Parameters parameters;
    parameters.probability = (argc > 7) ? std::stod(argv[7]) : 0.05;
    parameters.min_points = (argc > 8) ? (points.size() * std::stod(argv[8])) / 100 : (points.size() * 0.01);
    parameters.epsilon = (argc > 9) ? std::stod(argv[9]) : -1;
    parameters.cluster_epsilon = (argc > 10) ? std::stod(argv[10]) : -1;
    parameters.normal_threshold = (argc > 11) ? std::stod(argv[11]) : 0.9;
    int iteration_count = (argc > 12) ? std::stoi(argv[12]) : 3;

    // Plane detection using Efficient RANSAC
    t.reset();
    Efficient_ransac ransac;
    ransac.set_input(points);
    ransac.add_shape_factory<Plane>();
    std::cout << "Extracting planes...";
    // Adapted from https://doc.cgal.org/latest/Shape_detection/Shape_detection_2efficient_RANSAC_with_point_access_8cpp-example.html
    // Loop through RANSAC result and select the one with best coverage
    FT best_coverage = 0;
    Efficient_ransac::Shape_range shapes = ransac.shapes();
    std::cout << "Running RANSAC with following parameters" << std::endl;
    std::cout << "- probability: " << parameters.probability << std::endl;
    std::cout << "- min_points: " << parameters.min_points << std::endl;
    std::cout << "- normal_threshold: " << parameters.normal_threshold << std::endl;
    std::cout << "- cluster epsilon: " << parameters.cluster_epsilon << std::endl;
    std::cout << "- epsilon: " << parameters.epsilon << std::endl;
    CGAL::Timer tr;
    for (std::size_t i = 0; i < iteration_count; ++i)
    {
        tr.reset();
        tr.start();
        // Detect shapes.
        ransac.detect(parameters);
        // Measure time after detection.
        tr.stop();
        // Compute coverage, i.e. ratio of the points assigned to a shape.
        FT coverage = FT(points.size() - ransac.number_of_unassigned_points()) / FT(points.size());
        // Print number of assigned shapes and unassigned points.
        std::cout << ransac.shapes().end() - ransac.shapes().begin()
            << " primitives found, " << coverage << " coverage" << std::endl;
        std::cout << "Detection time: " << tr.time() * 1000 << "ms" << std::endl;
        // Choose result with the highest coverage.
        if (coverage > best_coverage)
        {
            best_coverage = coverage;
            // Efficient_ransac::shapes() provides
            // an iterator range to the detected shapes.
            shapes = ransac.shapes();
        }
    }
    Efficient_ransac::Plane_range planes = ransac.planes();
    std::size_t num_planes = planes.size();
    std::cout << "Done planes extraction. " << num_planes << " planes extracted. Time: " << t.time() << " sec." << std::endl;

    // Stores the plane index of each point as the third element of the tuple.
    Point_to_shape_index_map shape_index_map(points, planes);
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        // Uses the get function from the property map that accesses the 3rd element of the tuple.
        int plane_index = get(shape_index_map, i);
        points[i].get<2>() = plane_index;
    }

    // Use Polyfit to generate candidates faces
    std::cout << "Generating candidate faces...";
    t.reset();
    Polygonal_surface_reconstruction algo(
        points,
        Point_map(),
        Normal_map(),
        Plane_index_map());
    std::cout << " Done candidate faces. Time: " << t.time() << " sec." << std::endl;
    
    Surface_mesh model;

    // Use Polyfit to reconstruct mesh
    std::cout << "Reconstructing...";
    t.reset();
    if (!algo.reconstruct<MIP_Solver>(model, wt_fitting, wt_coverage, wt_complexity))
    {
        std::cerr << " Failed: " << algo.error_message() << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "Done polyfit reconstruction. Time: " << t.time() << " sec." << std::endl;

    // Cleanup and fix obtained mesh (holes, orientation, ...)
    t.reset();
    std::vector<Point> _points;
    std::vector<std::vector<std::size_t>> _polygons;
    Visitor visitor;
    Surface_mesh mesh;
    PMP::polygon_mesh_to_polygon_soup(model, _points, _polygons);
    PMP::repair_polygon_soup(_points, _polygons);
    PMP::orient_polygon_soup(_points, _polygons, CGAL::parameters::visitor(visitor));
    PMP::polygon_soup_to_polygon_mesh(_points, _polygons, mesh);
    PMP::triangulate_faces(mesh);
    if (CGAL::is_closed(mesh))
    {
        std::cout << "The obtained mesh is closed" << std::endl;
        PMP::orient_to_bound_a_volume(mesh);
    }
    std::cout << "Done mesh cleanup. Time: " << t.time() << " sec." << std::endl;


    // Write result mesh to file
    std::ofstream out(output_file);
    if (CGAL::IO::write_PLY(out, mesh))
    {
        std::cout << "Done writing to file. Saved to " << output_file << ". Time: " << t.time() << " sec." << std::endl;
    } 
    else
    {
        std::cerr << " Failed saving psr ransac result." << std::endl;
        return EXIT_FAILURE;
    }

    // Stores the candidate faces in a surface mesh and write it to file
    t.reset();
    Surface_mesh candidate_faces;
    algo.output_candidate_faces(candidate_faces);
    std::ofstream candidate_stream(candidate_faces_file);
    if (CGAL::IO::write_PLY(candidate_stream, candidate_faces))
        std::cout << "Candidate faces saved to " << candidate_faces_file << "." << std::endl;
    std::cout << " Done writing candidates faces. Time: " << t.time() << " sec." << std::endl;

    std::cout << "=> CGAL PSR RANSAC example finished\n"<< std::endl;

    return EXIT_SUCCESS;
}
#else
int main(int, char **)
{
    std::cerr << "This test requires either GLPK or SCIP to work.\n";
    return EXIT_SUCCESS;
}
#endif // defined(CGAL_USE_GLPK) || defined(CGAL_USE_SCIP)
