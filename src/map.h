#pragma once

#include "helper.h"

class Map {
public:
  Map(const std::string& map_file);

  ~Map() {}

//  int ClosestWaypoint(double x, double y) const;

//  int NextWaypoint(double x, double y, double theta) const;

//  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
//  std::vector<double> getFrenet(double x, double y, double theta) const;

//  // Transform from Frenet s,d coordinates to Cartesian x,y
//  std::vector<double> getXY(double s, double d) const;

  double getCenterD(const int lane_id) const;

  int getLaneId(const double d) const;

  std::vector<double> getXY(const double s, double d) const;

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
