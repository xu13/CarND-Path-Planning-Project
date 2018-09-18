#pragma once

#include "helper.h"

class Map {
public:
  Map(const std::string& map_file);

  ~Map() {}

  double getCenterD(const int lane_id) const;

  int getLaneId(const double d) const;

  /**
   * @brief Check if given d position is in given lane id.
   * @param d
   * @param lane id
   * @return
   */
  bool inLane(const double d, const int id) const;

  std::vector<double> getXY(const double s, double d) const;

  /**
   * @brief Compute the valid s position by wrapping over.
   * @param s
   * @return valid s
   */
  double getValidS(const double s) const { return std::fmod(s, MAX_S); }

  double getMaxS() const { return MAX_S; }

  double getLaneWidth() const { return LANE_WIDTH; }

  /**
   * @brief Get the yaw value of the road a specified s.
   * @param s
   * @return theta
   */
  double getYaw(const double s) const;

  /**
   * @brief Get velocity in Frenet coordinates.
   * @param vx
   * @param vy
   * @param s
   * @return {vs, vd}
   */
  std::vector<double> getVelocityFrenet(const double vx, const double vy, const double s) const;

private:
  // Map info
  std::vector<double> maps_x;
  std::vector<double> maps_y;
  std::vector<double> maps_s;
  std::vector<double> maps_dx;
  std::vector<double> maps_dy;
  // The max s value before wrapping around the track back to 0
  const double MAX_S = 6945.554;
  const double LANE_WIDTH = 4.0;

  tk::spline s_x;
  tk::spline s_y;
  tk::spline s_dx;
  tk::spline s_dy;
};
