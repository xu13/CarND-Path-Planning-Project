#pragma once

#include "helper.h"
#include "map.h"
#include "car.h"
#include "object.h"
#include "trajectory.h"

enum LaneChangeDir {
  LEFT = -1,
  RIGHT = 1
};

class Planner {
public:
  Planner(Map* map, Car* car);

  ~Planner() {}

  void plan(const std::unordered_map<int, Object>& objects, const size_t prev_size,
            std::vector<double>* next_x_vals, std::vector<double>* next_y_vals);

private:
  std::vector<double> quinticPolynomialSolver(const std::vector<double>& start,
                                              const std::vector<double>& end,
                                              const double T) const;
  double getDistanceQuintic(const std::vector<double>& coeff, const double t) const;
  double getVelocityQuintic(const std::vector<double> &coeff, const double t) const;
  double getAccelerationQuintic(const std::vector<double> &coeff, const double t) const;

  std::vector<double> quarticPolynomialSolver(const std::vector<double>& start,
                                              const std::vector<double>& end,
                                              const double T) const;
  double getDistanceQuartic(const std::vector<double>& coeff, const double t) const;
  double getVelocityQuartic(const std::vector<double> &coeff, const double t) const;
  double getAccelerationQuartic(const std::vector<double> &coeff, const double t) const;

  void computeVelocityKeepingTrajectory(const std::vector<double>& s_start,
                                        const std::vector<double>& d_start,
                                        size_t num_waypoints_to_be_added,
                                        Trajectory* trajectory) const;

  void computeFollowingTrajectory(const std::vector<double>& s_start,
                                  const std::vector<double>& d_start,
                                  const Object& front_car,
                                  size_t num_waypoints_to_be_added,
                                  Trajectory* trajectory) const;

  void computeChangingLaneTrajectory(const std::vector<double>& s_start,
                                     const std::vector<double>& d_start,
                                     size_t num_waypoints_to_be_added,
                                     Trajectory* trajectory) const;

private:
  Map* map_;
  Car* car_;
//  int lane_id_;
  const unsigned int N = 50;
  const double DT = 0.02;
  const double SPEED_LIMIT = 20.0;
  const double REF_SPEED = 20.0;
  const unsigned int LOOKAHEAD_SIZE = 5;
};
