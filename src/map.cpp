#include "map.h"

Map::Map(const std::string& map_file)
{
  std::ifstream in_map_(map_file.c_str(), std::ifstream::in);

  std::string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    double s;
    double d_x;
    double d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    maps_x_.push_back(x);
    maps_y_.push_back(y);
    maps_s_.push_back(s);
    maps_dx_.push_back(d_x);
    maps_dy_.push_back(d_y);
  }

  // Add the last point to the beginning of the list
  maps_s_.insert(maps_s_.begin(), maps_s_.back() - MAX_S);
  maps_x_.insert(maps_x_.begin(), maps_x_.back());
  maps_y_.insert(maps_y_.begin(), maps_y_.back());
  maps_dx_.insert(maps_dx_.begin(), maps_dx_.back());
  maps_dy_.insert(maps_dy_.begin(), maps_dy_.back());

  // Add the originally beginning two points (now index 1-2) to the end of the list
  for (size_t i = 1; i <= 2; i++) {
    maps_s_.push_back(MAX_S + maps_s_[i]);
    maps_x_.push_back(maps_x_[i]);
    maps_y_.push_back(maps_y_[i]);
    maps_dx_.push_back(maps_dx_[i]);
    maps_dy_.push_back(maps_dy_[i]);
  }

  // Set points of splines
  s_x_.set_points(maps_s_, maps_x_);
  s_y_.set_points(maps_s_, maps_y_);
  s_dx_.set_points(maps_s_, maps_dx_);
  s_dy_.set_points(maps_s_, maps_dy_);
}

double Map::getCenterD(const int lane_id) const
{
  return LANE_WIDTH / 2.0 + lane_id * LANE_WIDTH;
}

int Map::getLaneId(const double d) const
{
  return static_cast<int>(d / LANE_WIDTH);
}

bool Map::isInLane(const double d, const int id) const
{
  return std::abs(getCenterD(id) - d) <= (LANE_WIDTH / 4);
}

std::vector<double> Map::getXY(const double s, double d) const
{
  double s1 = std::fmod(s, MAX_S);
  double x = s_x_(s1);
  double y = s_y_(s1);
  double dx = s_dx_(s1);
  double dy = s_dy_(s1);
  return {x + d * dx, y + d * dy};
}

double Map::getYaw(const double s) const
{
  double dx = s_dx_(s);
  double dy = s_dy_(s);
  // dx and dy values define the unit normal vector pointing outward of the highway loop
  double theta = std::atan2(dy, dx) + M_PI_2;
  while (theta > M_PI) {
    theta -= 2 * M_PI;
  }
  while (theta < -M_PI) {
    theta += 2 * M_PI;
  }
  return theta;
}

std::vector<double> Map::getVelocityFrenet(const double vx, const double vy, const double s) const
{
  // Get lane yaw value
  double theta = getYaw(s);
  double vs = vx * std::cos(theta) + vy * std::sin(theta);
  double vd = vx * std::sin(theta) - vy * std::cos(theta);
  return {vs, vd};
}
