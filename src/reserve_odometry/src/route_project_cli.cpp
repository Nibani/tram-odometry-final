// Offline TRAIN projector. Input: x y z heading_x heading_y heading_z.
// Output: route_s cost distance base_x base_y base_z heading_cosine.
// Compile with the release's reserve_odometry/route.hpp; no ROS dependencies.
#include "reserve_odometry/route.hpp"
#include <iomanip>
#include <iostream>
#include <locale>
#include <sstream>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: route_project_cli BASELINE_LOOP.csv\n";
    return 2;
  }
  try {
    using namespace reserve_odometry;
    std::cin.imbue(std::locale::classic());
    std::cout.imbue(std::locale::classic());
    Route route = Route::load(argv[1], 7.55, true);
    std::cout << std::setprecision(17);
    std::string line;
    while (std::getline(std::cin, line)) {
      if (line.empty()) continue;
      std::istringstream input(line);
      input.imbue(std::locale::classic());
      Vec3 point, heading;
      std::string extra;
      if (!(input >> point[0] >> point[1] >> point[2]
                  >> heading[0] >> heading[1] >> heading[2]) ||
          (input >> extra) || !finite(point) || !finite(heading)) {
        throw std::runtime_error("Expected six finite projection inputs");
      }
      auto projection = route.project(point, &heading);
      auto body = route.pose(projection.s);
      std::cout << projection.s << ' ' << projection.cost << ' '
                << projection.distance << ' ' << body.position[0] << ' '
                << body.position[1] << ' ' << body.position[2] << ' '
                << dot(heading, body.forward) << '\n';
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
