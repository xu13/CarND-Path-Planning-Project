#include "map.h"

int main(int argc, char** argv)
{
  Map map("../data/highway_map.csv");

  std::ofstream myfile;
  myfile.open ("../data/highway_map_high_precision.csv");

  for (double s = 0; s < map.getMaxS(); s += 1) {
    for (unsigned int lane_id = 0; lane_id <= 3; lane_id++) {
      double d = lane_id * map.getLaneWidth();
      std::vector<double> xy = map.getXY(s, d);
      myfile << xy[0] << " " << xy[1] << " ";
    }
    myfile << "\n";
  }
  for (unsigned int lane_id = 0; lane_id <= 3; lane_id++) {
    double d = lane_id * map.getLaneWidth();
    std::vector<double> xy = map.getXY(map.getMaxS(), d);
    myfile << xy[0] << " " << xy[1] << " ";
  }

  myfile.close();


  return 0;
}

