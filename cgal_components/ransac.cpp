// A simple Efficient RANSAC example to understand its inner working
// Produce colored point cloud with one color for each detected plane
// Adapted from https://doc.cgal.org/latest/Shape_detection/Shape_detection_2efficient_RANSAC_with_point_access_8cpp-example.html
// Documentation: https://doc.cgal.org/latest/Shape_detection/index.html#title1

#include <fstream>
#include <iostream>
#include <string>
#include <CGAL/Timer.h>
#include <CGAL/number_utils.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/IO/write_ply_points.h>

// Type declarations.
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3> Point_with_normal;
typedef std::vector<Point_with_normal> Pwn_vector;
typedef CGAL::First_of_pair_property_map<Point_with_normal> Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
typedef CGAL::Shape_detection::Efficient_RANSAC_traits<Kernel, Pwn_vector, Point_map, Normal_map> Traits;
typedef CGAL::Shape_detection::Efficient_RANSAC<Traits> Efficient_ransac;
typedef CGAL::Shape_detection::Plane<Traits> Plane;

typedef std::array<unsigned char, 3> Color;
typedef std::tuple<Kernel::Point_3, Kernel::Vector_3, Color> PNC;
typedef CGAL::Nth_of_tuple_property_map<0, PNC> p_map;
typedef CGAL::Nth_of_tuple_property_map<1, PNC> n_map;
typedef CGAL::Nth_of_tuple_property_map<2, PNC> c_map;

// Necessary to handle color information when writing ply
namespace CGAL
{
    template <class F>
    struct Output_rep<::Color, F>
    {
        const ::Color &c;
        static const bool is_specialized = true;
        Output_rep(const ::Color &c) : c(c)
        {
        }
        std::ostream &operator()(std::ostream &out) const
        {
            if (IO::is_ascii(out))
                out << int(c[0]) << " " << int(c[1]) << " " << int(c[2]);
            else
                out.write(reinterpret_cast<const char *>(&c), sizeof(c));
            return out;
        }
    };
} // namespace CGAL

int main(int argc, char **argv)
{
    std::cout << "=> CGAL RANSAC example started" << std::endl;

    // Points with normal container for input points
    Pwn_vector points;

    // Load points with normal from input file
    if (!CGAL::IO::read_points(
            ((argc > 1) ? argv[1] : "results/dsm_as_point_cloud.ply"),
            std::back_inserter(points),
            CGAL::parameters::point_map(Point_map()).normal_map(Normal_map())))
    {
        std::cerr << "Error: cannot read input file" << std::endl;
        return EXIT_FAILURE;
    }
    else
    {
        std::cout << "Done input points loading. " << points.size() << " points loaded." << std::endl;
    }

    // Instantiate shape detection engine.
    Efficient_ransac ransac;

    // Provide input data.
    ransac.set_input(points);

    // Register detection of planes
    // More primitives are available if needed (sphere, torus, etc)
    ransac.add_shape_factory<Plane>();

    // Set parameters for shape detection.
    Efficient_ransac::Parameters parameters;

    // Sets probability to miss the largest primitive at each iteration.
    // A lower probability provides a higher reliability and determinism at the cost
    // of longer running time due to a higher search endurance.
    // It must belong to the interval [0, 1].
    // default value in CGAL is 0.05
    parameters.probability = (argc > 3) ? std::stod(argv[3]) : 0.05;

    // Sets minimum number of points in a detected shape as a percentage of input points
    // Default value in CGAL is 1% ot total number of input points
    // It must belong to the interval [0, +inf).
    parameters.min_points = (argc > 4) ? (points.size() * std::stod(argv[4])) / 100 : (points.size() * 0.01);

    // Sets maximum acceptable Euclidean distance between a point and a shape.
    // Default value in CGAL is 1% of bounding box diagonal
    // It must belong to the interval [0, +inf).
    parameters.epsilon = (argc > 5) ? std::stod(argv[5]) : -1;

    // Set maximum acceptable Euclidean distance between points, which are assumed to be neighbors.
    // Default CGAL value is 1% of the bounding box diagonal.
    // TODO: calculate this
    parameters.cluster_epsilon = (argc > 6) ? std::stod(argv[6]) : -1;

    // Set maximum threshold on the dot product between the estimated
    // shape's normal and the point's normal, that is the cosine of the angle (cos(25°) = 0.9).
    // dot(surface_normal, point_normal);
    // Default CGAL value is 0.9 (around 25 degrees).
    // It must belong to the interval [0, +inf).
    parameters.normal_threshold = (argc > 7) ? std::stod(argv[7]) : 0.9;

    // Number of Efficient RANSAC iteration
    int iteration_count = (argc > 8) ? std::stoi(argv[8]) : 3;

    // Measure time before setting up the shape detection.
    CGAL::Timer time;
    time.start();

    // Constructs internal data structures required for the shape detection.
    // These structures only depend on the input data, i.e. the points and normal vectors.
    // This method is called by detect(), if it was not called before by the user.
    ransac.preprocess();

    // Measure time after preprocessing.
    time.stop();
    std::cout << "Input points preprocessing took: " << time.time() * 1000 << " ms." << std::endl;

    // Perform detection several times and choose result with the highest coverage.
    Efficient_ransac::Shape_range shapes = ransac.shapes();
    std::cout << "Running RANSAC with following parameters" << std::endl;
    std::cout << "- probability: " << parameters.probability << std::endl;
    std::cout << "- min_points: " << parameters.min_points << std::endl;
    std::cout << "- normal_threshold: " << parameters.normal_threshold << std::endl;
    std::cout << "- cluster epsilon: " << parameters.cluster_epsilon << std::endl;
    std::cout << "- epsilon: " << parameters.epsilon << std::endl;

    FT best_coverage = 0;
    for (std::size_t i = 0; i < iteration_count; ++i)
    {
        // Reset timer.
        time.reset();
        time.start();
        // Detect shapes.
        // Shape types considered during the detection are those registered using add_shape_factory().
        ransac.detect(parameters);
        // Measure time after detection.
        time.stop();
        // Compute coverage, i.e. ratio of the points assigned to a shape.
        FT coverage =
            FT(points.size() - ransac.number_of_unassigned_points()) / FT(points.size());
        // Print number of assigned shapes and coverage with elapsed time.
        std::cout << ransac.shapes().end() - ransac.shapes().begin()
            << " primitives found, " << coverage << " coverage" << std::endl;
        std::cout << "Detection time: " << time.time() * 1000 << "ms" << std::endl;

        // Choose result with the highest coverage.
        if (coverage > best_coverage)
        {
            best_coverage = coverage;
            // Efficient_ransac::shapes() provides
            // an iterator range to the detected shapes.
            shapes = ransac.shapes();
        }
    }

    int shapes_nb = ransac.shapes().end() - ransac.shapes().begin();
    std::vector<PNC> classified_points; // store points
    Efficient_ransac::Shape_range::iterator it = shapes.begin();
    CGAL::Random rand(static_cast<unsigned int>(shapes_nb));

    while (it != shapes.end())
    {
        boost::shared_ptr<Efficient_ransac::Shape> shape = *it;
        // Use Shape_base::info() to print the parameters of the detected shape.
        std::cout << (*it)->info() << std::endl;

        unsigned char r = static_cast<unsigned char>(64 + rand.get_int(0, 192));
        unsigned char g = static_cast<unsigned char>(64 + rand.get_int(0, 192));
        unsigned char b = static_cast<unsigned char>(64 + rand.get_int(0, 192));
        const Color &c = CGAL::make_array(r, g, b);

        // Iterate through point indices assigned to each detected shape.
        std::vector<std::size_t>::const_iterator
            index_it = (*it)->indices_of_assigned_points().begin();
        while (index_it != (*it)->indices_of_assigned_points().end())
        {
            // Retrieve point.
            const Point_with_normal &p = *(points.begin() + (*index_it));
            classified_points.push_back(std::make_tuple(p.first, p.second, c));
            // Proceed with the next point.
            index_it++;
        }
        // Proceed with the next detected shape.
        it++;
    }

    const char *output_filename = (argc > 2) ? argv[2] : "results/ransac.ply";
    std::ofstream ofile(output_filename);
    CGAL::IO::write_PLY_with_properties(ofile, classified_points,
                                        CGAL::make_ply_point_writer(p_map()),
                                        CGAL::make_ply_normal_writer(n_map()),
                                        std::make_tuple(c_map(),
                                                        CGAL::IO::PLY_property<unsigned char>("red"),
                                                        CGAL::IO::PLY_property<unsigned char>("green"),
                                                        CGAL::IO::PLY_property<unsigned char>("blue")));

    std::cout << "=> CGAL RANSAC example finished\n"
              << std::endl;

    return EXIT_SUCCESS;
}