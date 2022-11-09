// STL includes.
#include <string>
#include <vector>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>

// Boost includes.
#include <boost/iterator/function_output_iterator.hpp>

// CGAL includes.
#include <CGAL/Timer.h>
#include <CGAL/Random.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>

#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing_on_point_set.h>

// Type declarations.
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;

using FT = typename Kernel::FT;
using Point_3 = typename Kernel::Point_3;
using Vector_3 = typename Kernel::Vector_3;

using Input_range = CGAL::Point_set_3<Point_3>;
using Point_map = typename Input_range::Point_map;
using Normal_map = typename Input_range::Vector_map;

using Neighbor_query = CGAL::Shape_detection::Point_set::K_neighbor_query<Kernel, Input_range, Point_map>;
using Region_type = CGAL::Shape_detection::Point_set::Least_squares_plane_fit_region<Kernel, Input_range, Point_map, Normal_map>;
using Region_growing = CGAL::Shape_detection::Region_growing<Input_range, Neighbor_query, Region_type>;

using Indices = std::vector<std::size_t>;
using Output_range = CGAL::Point_set_3<Point_3>;
using Points_3 = std::vector<Point_3>;

// Define an insert iterator for colouring point data
struct Insert_point_colored_by_region_index
{
    using argument_type = Indices;
    using result_type = void;

    using Color_map =
        typename Output_range::template Property_map<unsigned char>;

    const Input_range &m_input_range;
    const Point_map m_point_map;
    Output_range &m_output_range;
    std::size_t &m_number_of_regions;

    Color_map m_red, m_green, m_blue;

    Insert_point_colored_by_region_index(
        const Input_range &input_range,
        const Point_map point_map,
        Output_range &output_range,
        std::size_t &number_of_regions) : m_input_range(input_range),
                                          m_point_map(point_map),
                                          m_output_range(output_range),
                                          m_number_of_regions(number_of_regions)
    {

        m_red =
            m_output_range.template add_property_map<unsigned char>("red", 0).first;
        m_green =
            m_output_range.template add_property_map<unsigned char>("green", 0).first;
        m_blue =
            m_output_range.template add_property_map<unsigned char>("blue", 0).first;
    }

    result_type operator()(const argument_type &region)
    {

        CGAL::Random rand(static_cast<unsigned int>(m_number_of_regions));
        const unsigned char r =
            static_cast<unsigned char>(64 + rand.get_int(0, 192));
        const unsigned char g =
            static_cast<unsigned char>(64 + rand.get_int(0, 192));
        const unsigned char b =
            static_cast<unsigned char>(64 + rand.get_int(0, 192));

        for (const std::size_t index : region)
        {
            const auto &key = *(m_input_range.begin() + index);

            const Point_3 &point = get(m_point_map, key);
            const auto it = m_output_range.insert(point);

            m_red[*it] = r;
            m_green[*it] = g;
            m_blue[*it] = b;
        }
        ++m_number_of_regions;
    }
}; // Insert_point_colored_by_region_index end

// MAIN METHOD
int main(int argc, char *argv[])
{

    std::cout << "=> CGAL Region Growing on Points example started"
              << std::endl;

    // Load xyz data either from a local folder or a user-provided file.
    const char *filename = (argc > 1) ? argv[1] : "results/dsm_as_point_cloud.ply";
    std::ifstream in(filename);
    CGAL::IO::set_ascii_mode(in);

    if (!in)
    {
        std::cout << "Error: cannot read the file " << filename << std::endl;
        std::cout << "You can either create a symlink to the data folder or provide this file by hand."
                  << std::endl;
        return EXIT_FAILURE;
    }

    const bool with_normal_map = true;
    Input_range input_range(with_normal_map);

    in >> input_range;
    in.close();

    std::cout << "Loaded input file with "
              << input_range.size() << " points with normals"
              << std::endl;

    // Region growing variables
    // CGAL default is 1
    FT max_distance_to_plane = (argc > 3) ? FT(std::stof(argv[3])) : FT(1);
    // CGAL default is 25
    FT max_accepted_angle = (argc > 4) ? FT(std::stof(argv[4])) : FT(25);
    // CGAL default is 3 ???
    std::size_t min_region_size = (argc > 5) ? ((input_range.size() * std::stoi(argv[5])) / 100) : 3;
    // CGAL default value is 12 for nearest neighbour K
    std::size_t k = (argc > 6) ? std::stoi(argv[6]) : 12;

    // Create instances of the classes Neighbor_query and Region_type.
    Neighbor_query neighbor_query(
        input_range,
        k,
        input_range.point_map());

    // region_type uses leasts squares to fit plane to region
    Region_type region_type(
        input_range,
        max_distance_to_plane, max_accepted_angle, min_region_size,
        input_range.point_map(), input_range.normal_map());

    // Create an instance of the region growing class.
    Region_growing region_growing(
        input_range, neighbor_query, region_type);

    // Run the algorithm.
    Output_range output_range;
    std::size_t number_of_regions = 0;

    // Colorize result
    Insert_point_colored_by_region_index inserter(
        input_range, input_range.point_map(),
        output_range, number_of_regions);

    CGAL::Timer timer;

    timer.start();
    region_growing.detect(
        boost::make_function_output_iterator(inserter));
    timer.stop();

    // Print the number of found regions.
    std::cout << number_of_regions << " regions have been found in " << timer.time() << " seconds"
              << std::endl;

    // Save the result to a file
    const std::string output_filename = (argc > 2) ? argv[2] : "results/region_growing_points.ply";
    std::ofstream out(output_filename);
    out << output_range;
    std::cout << "Found regions have been saved in " << output_filename << std::endl;
    out.close();

    // Get all unassigned items.
    Indices unassigned_items;
    region_growing.unassigned_items(std::back_inserter(unassigned_items));

    // Print the number of unassigned items.
    std::cout << unassigned_items.size() << " points do not belong to any region"
              << std::endl;

    // Store all unassigned points.
    Points_3 unassigned_points;
    unassigned_points.reserve(unassigned_items.size());

    for (const auto index : unassigned_items)
    {
        const auto &key = *(input_range.begin() + index);

        const Point_3 &point = get(input_range.point_map(), key);
        unassigned_points.push_back(point);
    }

    std::cout << unassigned_points.size() << " unassigned points are stored"
              << std::endl;

    std::cout << "=> CGAL Region Growing on Points example finished\n"
              << std::endl;

    return EXIT_SUCCESS;
}
