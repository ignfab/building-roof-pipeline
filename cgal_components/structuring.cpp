#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/IO/write_points.h>
#include <CGAL/property_map.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/structure_point_set.h>
#include <CGAL/Surface_mesh.h>
#include <iostream>
#include <fstream>

// Type declarations
typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef Kernel::Point_3                                      Point;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3>         Point_with_normal;
typedef std::vector<Point_with_normal>                       Pwn_vector;
typedef CGAL::First_of_pair_property_map<Point_with_normal>  Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
// Efficient RANSAC types
typedef CGAL::Shape_detection::Efficient_RANSAC_traits
  <Kernel, Pwn_vector, Point_map, Normal_map>              Traits;
typedef CGAL::Shape_detection::Efficient_RANSAC<Traits>    Efficient_ransac;
typedef CGAL::Shape_detection::Plane<Traits>               Plane;
typedef Kernel::Point_3                                    Point_3;
typedef CGAL::Surface_mesh<Point_3>                        Surface_mesh;

int main (int argc, char** argv)
{
  std::cout << "=> CGAL Structuring example started" << std::endl;

  const char* input_file = (argc>1) ? argv[1] : "results/dsm_as_point_cloud.ply";
  const char* output_file = (argc>2) ? argv[2] : "results/structuring.ply";
  const float epsilon = (argc>3) ? std::atof(argv[3]) : 0.98;
  std::string temp_output_file(output_file);
  temp_output_file.append(".tmp");
  
  // Points with normals.
  Pwn_vector points;
  // Loading point set from a file.
  if(!CGAL::IO::read_points(input_file, std::back_inserter(points),
                            CGAL::parameters::point_map(Point_map())
                                             .normal_map(Normal_map())))
  {
    std::cerr << "Error: cannot read file " << input_file << std::endl;
    return EXIT_FAILURE;
  }
  std::cerr << points.size() << " point(s) read." << std::endl;
  
  // Shape detection
  Efficient_ransac ransac;
  ransac.set_input(points);
  ransac.add_shape_factory<Plane>();
  ransac.detect();
  Efficient_ransac::Plane_range planes = ransac.planes();
  Pwn_vector structured_pts;
  CGAL::structure_point_set(points,
                            planes,
                            std::back_inserter(structured_pts),
                            epsilon, // HUGE epsilon for structuring points
                            CGAL::parameters::point_map(Point_map())
                                             .normal_map(Normal_map())
                                             .plane_map(CGAL::Shape_detection::Plane_map<Traits>())
                                             .plane_index_map(CGAL::Shape_detection::Point_to_shape_index_map<Traits>(points, planes)));
  std::cerr << structured_pts.size ()
            << " structured point(s) generated." << std::endl;

  CGAL::IO::write_PLY(temp_output_file, structured_pts,
                         CGAL::parameters::point_map(Point_map())
                                          .normal_map(Normal_map())
                                          .stream_precision(17));                

    Surface_mesh surface_mesh;
    std::ifstream is(temp_output_file);
    if (!is || !CGAL::IO::read_PLY(is, surface_mesh)) {
        std::cerr << "Failed to read output file : " << temp_output_file << std::endl;
        return EXIT_FAILURE;
    }
    CGAL::IO::write_polygon_mesh(output_file, surface_mesh, CGAL::parameters::stream_precision(17));
    remove(temp_output_file.c_str());

  std::cout << "=> CGAL Structuring example finished\n" << std::endl;

  return EXIT_SUCCESS;
}