#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>

#include <chrono>
#include <iostream>

using namespace std;

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> Surface_mesh;

namespace SMS = CGAL::Surface_mesh_simplification;

/**
 * @brief 
 *
 * The following example illustrates the simplification of a Surface_mesh. 
 * The unspecified cost strategy defaults to Lindstrom-Turk.
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */

int main(int argc, char **argv)
{

  std::cout << "=> CGAL Edge Collapse example started" << std::endl;

  /* Variables initialization */

  Surface_mesh surface_mesh;

  const char *filename = (argc > 1) ? argv[1] : "results/dsm_as_mesh.ply";
  const char* output_filename = (argc > 2) ? argv[2] : "results/edge_collapse.ply";

  // In this example, the simplification stops when the number of undirected edges
  // drops below 10% of the initial count
  double stop_ratio = (argc > 3) ? stod(argv[3]) : 0.1;
  SMS::Count_ratio_stop_predicate<Surface_mesh> stop(stop_ratio);

  ifstream is(filename);
  if (!is || !CGAL::IO::read_PLY(is, surface_mesh))
  {
    cerr << "Failed to read input mesh: " << filename << endl;
    return EXIT_FAILURE;
  }
  if (!CGAL::is_triangle_mesh(surface_mesh))
  {
    cerr << "Input geometry is not triangulated." << endl;
    return EXIT_FAILURE;
  }

  /* Start processing  */
  
  cout << "Starting rebuild..." << endl;

  chrono::steady_clock::time_point start_time = chrono::steady_clock::now();

  int r = SMS::edge_collapse(surface_mesh, stop);

  chrono::steady_clock::time_point end_time = chrono::steady_clock::now();

  cout << "Rebuild finished!\n"
       << r << " edges removed.\n"
       << surface_mesh.number_of_edges() << " final edges.\n";
  cout << "Time elapsed: " << chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count() << "ms" << endl;

  CGAL::IO::write_polygon_mesh(output_filename, surface_mesh, CGAL::parameters::stream_precision(17));

  std::cout << "=> CGAL Edge Collapse example finished\n" << std::endl;

  return EXIT_SUCCESS;
}
