#pragma once

#include "helper.h"

/**
 * @brief The Map class.
 */
class Map {
public:
  Map(const std::string& map_file);

  ~Map() {}

  /**
   * @brief Get the d value for the lane center.
   * @param lane_id
   * @return
   */
  double getCenterD(const int lane_id) const;

  /**
   * @brief Get the lane id given a d value.
   * @param d
   * @return
   */
  int getLaneId(const double d) const;

  /**
   * @brief Check if given d position is in given lane id.
   * @param d
   * @param lane id
   * @return
   */
  bool isInLane(const double d, const int id) const;

  /**
   * @brief Check if given lane is the left lane.
   * @param id
   * @return
   */
  bool isLeftLane(const int id) const { return id == 0; }

  /**
   * @brief Check if given lane is the middle lane.
   * @param id
   * @return
   */
  bool isMiddleLane(const int id) const { return id == 1; }

  /**
   * @brief Check if given lane is the right lane.
   * @param id
   * @return
   */
  bool isRightLane(const int id) const { return id == 2; }

  /**
   * @brief Get x, y coordinates given Frenet coordinates s, d.
   * @param s
   * @param d
   * @return
   */
  std::vector<double> getXY(const double s, double d) const;

  /**
   * @brief Compute the valid s position for cases where s is larger than MAX_S.
   * @param s
   * @return valid s
   */
  double getValidS(const double s) const { return std::fmod(s, MAX_S); }

  /**
   * @brief Get the loop s length.
   * @return
   */
  double getMaxS() const { return MAX_S; }

  /**
   * @brief Get lane width. Assuming every lane has the same width.
   * @return
   */
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
  std::vector<double> maps_x_;
  std::vector<double> maps_y_;
  std::vector<double> maps_s_;
  std::vector<double> maps_dx_;
  std::vector<double> maps_dy_;
  const double MAX_S = 6945.554; // maximum length of the track
  const double LANE_WIDTH = 4.0; // lane width

  tk::spline s_x_;
  tk::spline s_y_;
  tk::spline s_dx_;
  tk::spline s_dy_;
};
