#include "planner.h"

Planner::Planner(Map* map, Car* car)
{
  map_ = map;
  car_ = car;
}

void Planner::plan(const std::unordered_map<int, Object>& objects, const size_t leftover_size,
                   std::vector<double>* next_x_vals, std::vector<double>* next_y_vals) const
{
  std::cout << "Current car state: " << *car_ << std::endl;

  Trajectory trajectory;

  // Start waypoint for the next plan
  std::vector<double> s_start = {car_->getS(), 0.0, 0.0};
  std::vector<double> d_start = {car_->getD(), 0.0, 0.0};

  // If we already have previous plan, use the last point of that plan as a starting point for the new plan
  size_t num_waypoints_to_be_added = N;
  if (leftover_size > 0) {
    const size_t num_waypoints_consumed = N - leftover_size;

    // Start from this waypoint from previous trajectory
    const Waypoint& wp = car_->getPreviousTrajectory().getWaypoint(LOOKAHEAD_SIZE + num_waypoints_consumed);
    s_start[0] = wp.getS();
    s_start[1] = wp.getSdot();
    s_start[2] = wp.getSdotdot();
    d_start[0] = wp.getD();
    d_start[1] = wp.getDdot();
    d_start[2] = wp.getDdotdot();

    // Add the lookahead waypoints from the previous plan to the new plan
    for (size_t i = num_waypoints_consumed; i < num_waypoints_consumed + LOOKAHEAD_SIZE; i++) {
      const Waypoint& wp = car_->getPreviousTrajectory().getWaypoint(i);
      trajectory.addWaypoint(wp);
    }
    num_waypoints_to_be_added = N - LOOKAHEAD_SIZE;
  }

  // Loop through all objects (other cars) to find surrounding traffic
  std::pair<int, double> front_car = std::make_pair(-1, std::numeric_limits<double>::infinity());
  std::pair<int, double> front_left_car = std::make_pair(-1, std::numeric_limits<double>::infinity());
  std::pair<int, double> front_right_car = std::make_pair(-1, std::numeric_limits<double>::infinity());
  std::pair<int, double> rear_left_car = std::make_pair(-1, -std::numeric_limits<double>::infinity());
  std::pair<int, double> rear_right_car = std::make_pair(-1, -std::numeric_limits<double>::infinity());

  const int car_lane_id = map_->getLaneId(d_start[0]);
  for (auto& p : objects) {
    const int id = p.first;
    const Object& obj = p.second;

    // Lane id of the object
    const int obj_lane_id = map_->getLaneId(obj.getD());
    // This is the s position of the object after lookahead time (projection ahead)
    double obj_s = map_->getValidS(obj.getS() + LOOKAHEAD_SIZE * obj.getSdot() * DT);
    double ds = obj_s - s_start[0];
    // Corrections for beginning/ending of the loop
    if (ds > map_->getMaxS() / 2.0) {
      ds -= map_->getMaxS();
    } else if (ds < -map_->getMaxS() / 2.0) {
      ds += map_->getMaxS();
    }

    // We ignore any object far from the ego-car
    if (std::abs(ds) > 100.0) {
      continue;
    }

    // Find the info on surrounding cars
    if (obj_lane_id == car_lane_id) {
      if (ds > 0 && ds < front_car.second) {
        front_car.first = id;
        front_car.second = ds;
      }
    } else if (obj_lane_id == car_lane_id + LaneChangeDir::LEFT) {
      if (ds > 0 && ds < front_left_car.second) {
        front_left_car.first = id;
        front_left_car.second = ds;
      } else if (ds < 0 && ds > rear_left_car.second) {
        rear_left_car.first = id;
        rear_left_car.second = ds;
      }
    } else if (obj_lane_id == car_lane_id + LaneChangeDir::RIGHT) {
      if (ds > 0 && ds < front_right_car.second) {
        front_right_car.first = id;
        front_right_car.second = ds;
      } else if (ds < 0 && ds > rear_right_car.second) {
        rear_right_car.first = id;
        rear_right_car.second = ds;
      }
    }
  }

  // Print starting points
  std::cout << "start: s=" << s_start[0] << ", s_dot=" << s_start[1] << ", s_dot_dot=" << s_start[2] << "\n"
            << "       d=" << d_start[0] << ", d_dot=" << d_start[1] << ", d_dot_dot=" << d_start[2] << std::endl;

  // State machine
  switch (car_->getState()) {
    case State::VELOCITY_KEEPING: {
        computeVelocityKeepingTrajectory(s_start, d_start, num_waypoints_to_be_added, &trajectory);

        const double FRONT_SAFETY_DISTANCE = 30.0;
        if (front_car.second < FRONT_SAFETY_DISTANCE) {
          car_->setState(State::FOLLOWING);
        }
        break;
      }
    case State::FOLLOWING: {
        computeFollowingTrajectory(s_start, d_start,
                                   objects.at(front_car.first),
                                   num_waypoints_to_be_added, &trajectory);

        const double FRONT_SAFETY_DISTANCE = 40.0;
        const double REAR_SAFETY_DISTANCE = -10.0;
        if (map_->isMiddleLane(car_->getTargetLaneId()) &&
            front_left_car.second > FRONT_SAFETY_DISTANCE &&
            rear_left_car.second < REAR_SAFETY_DISTANCE &&
            front_right_car.second > FRONT_SAFETY_DISTANCE &&
            rear_right_car.second < REAR_SAFETY_DISTANCE) {
          if (front_left_car.second >= front_right_car.second) {
            car_->setTargetLaneId(car_->getTargetLaneId() + LaneChangeDir::LEFT);
            car_->setState(State::CHANGE_LANE_LEFT);
          } else {
            car_->setTargetLaneId(car_->getTargetLaneId() + LaneChangeDir::RIGHT);
            car_->setState(State::CHANGE_LANE_RIGHT);
          }
        } else if (!map_->isLeftLane(car_->getTargetLaneId()) &&
                   front_left_car.second > FRONT_SAFETY_DISTANCE &&
                   rear_left_car.second < REAR_SAFETY_DISTANCE) {
          car_->setTargetLaneId(car_->getTargetLaneId() + LaneChangeDir::LEFT);
          car_->setState(State::CHANGE_LANE_LEFT);
        } else if (!map_->isRightLane(car_->getTargetLaneId()) &&
                   front_right_car.second > FRONT_SAFETY_DISTANCE &&
                   rear_right_car.second < REAR_SAFETY_DISTANCE) {
          car_->setTargetLaneId(car_->getTargetLaneId() + LaneChangeDir::RIGHT);
          car_->setState(State::CHANGE_LANE_RIGHT);
        }
        break;
      }
    case State::CHANGE_LANE_LEFT: {
        computeChangingLaneTrajectory(s_start, d_start, num_waypoints_to_be_added, &trajectory);

        if (map_->isInLane(d_start[0], car_->getTargetLaneId())) {
          car_->setState(State::VELOCITY_KEEPING);
        }
        break;
      }
    case State::CHANGE_LANE_RIGHT: {
        computeChangingLaneTrajectory(s_start, d_start, num_waypoints_to_be_added, &trajectory);

        if (map_->isInLane(d_start[0], car_->getTargetLaneId())) {
          car_->setState(State::VELOCITY_KEEPING);
        }
        break;
      }
    default:
      break;
  }

  // Save the trajectory for next cycle use
  car_->setPreviousTrajectory(trajectory);

  // Fill out the output variables
  for (const Waypoint& wp : trajectory.getWaypoints()) {
    next_x_vals->push_back(wp.getX());
    next_y_vals->push_back(wp.getY());
  }

  // Print debugging info
//  std::cout << "Previous plan remaining size: " << leftover_size << std::endl;
//  std::cout << "Next plan size: " << num_waypoints_to_be_added << std::endl;
  std::cout << "Next state: " << state_str[car_->getState()] << std::endl;
  std::cout << "Current lane id: " << map_->getLaneId(d_start[0]) << std::endl;
  std::cout << "Target lane id: " << car_->getTargetLaneId() << std::endl;
  std::cout << "----------" << std::endl;

}

void Planner::computeVelocityKeepingTrajectory(const std::vector<double>& s_start,
                                               const std::vector<double>& d_start,
                                               size_t num_waypoints_to_be_added,
                                               Trajectory* trajectory) const
{
  double ref_v = SPEED_LIMIT;
  if (s_start[1] < SPEED_LIMIT) {
    ref_v = s_start[1] + 6.0;
  }
  const double T = 3;
  std::vector<double> s_end = {-1, ref_v, 0.0};
  std::vector<double> s_coeff = quarticPolynomialSolver(s_start, s_end, T);

  std::vector<double> d_end = {map_->getCenterD(car_->getTargetLaneId()), 0.0, 0.0};
  std::vector<double> d_coeff = quinticPolynomialSolver(d_start, d_end, T);

  std::cout << "end: s=" << s_end[0] << ", s_dot=" << s_end[1] << ", s_dot_dot=" << s_end[2] << "\n"
            << "     d=" << d_end[0] << ", d_dot=" << d_end[1] << ", d_dot_dot=" << d_end[2] << std::endl;
  std::cout << "T=" << T << std::endl;

  for (size_t i = 0; i < num_waypoints_to_be_added; i++) {
    double s = map_->getValidS(getDistanceQuartic(s_coeff, i * DT));
    double s_dot = getVelocityQuartic(s_coeff, i * DT);
    double s_dot_dot = getAccelerationQuartic(s_coeff, i * DT);
    double d = getDistanceQuartic(d_coeff, i * DT);
    double d_dot = getVelocityQuartic(d_coeff, i * DT);
    double d_dot_dot = getAccelerationQuartic(d_coeff, i * DT);
    std::vector<double> xy = map_->getXY(s, d);

    Waypoint wp(s, d, s_dot, d_dot, s_dot_dot, d_dot_dot);
    wp.setX(xy[0]);
    wp.setY(xy[1]);
    trajectory->addWaypoint(wp);
  }
}

void Planner::computeFollowingTrajectory(const std::vector<double>& s_start,
                                         const std::vector<double>& d_start,
                                         const Object& front_car,
                                         size_t num_waypoints_to_be_added,
                                         Trajectory* trajectory) const
{
  std::cout << "Front car: " << front_car << std::endl;
  const double T = 3;
  double end_s = front_car.getS() + front_car.getSdot() * T - (0 + 1.5 * front_car.getSdot());
  double end_s_dot = front_car.getSdot();

  std::vector<double> s_end = {end_s, end_s_dot, 0.0};
  std::vector<double> s_coeff = quinticPolynomialSolver(s_start, s_end, T);

  std::vector<double> d_end = {map_->getCenterD(car_->getTargetLaneId()), 0.0, 0.0};
  std::vector<double> d_coeff = quinticPolynomialSolver(d_start, d_end, T);

  std::cout << "end: s=" << s_end[0] << ", s_dot=" << s_end[1] << ", s_dot_dot=" << s_end[2] << "\n"
            << "     d=" << d_end[0] << ", d_dot=" << d_end[1] << ", d_dot_dot=" << d_end[2] << std::endl;
  std::cout << "T=" << T << std::endl;

  for (size_t i = 0; i < num_waypoints_to_be_added; i++) {
    double s = map_->getValidS(getDistanceQuintic(s_coeff, i * DT));
    double s_dot = getVelocityQuintic(s_coeff, i * DT);
    double s_dot_dot = getAccelerationQuintic(s_coeff, i * DT);
    double d = getDistanceQuartic(d_coeff, i * DT);
    double d_dot = getVelocityQuartic(d_coeff, i * DT);
    double d_dot_dot = getAccelerationQuartic(d_coeff, i * DT);
    std::vector<double> xy = map_->getXY(s, d);

    Waypoint wp(s, d, s_dot, d_dot, s_dot_dot, d_dot_dot);
    wp.setX(xy[0]);
    wp.setY(xy[1]);
    trajectory->addWaypoint(wp);
  }
}

void Planner::computeChangingLaneTrajectory(const std::vector<double>& s_start,
                                            const std::vector<double>& d_start,
                                            size_t num_waypoints_to_be_added,
                                            Trajectory* trajectory) const
{
  // For simplicity, we change lane with constant speed
  double ref_v = s_start[1];
  const double T = 3;

  std::vector<double> s_end = {-1, ref_v, 0};
  std::vector<double> s_coeff = quarticPolynomialSolver(s_start, s_end, T);

  std::vector<double> d_end = {map_->getCenterD(car_->getTargetLaneId()), 0, 0};
  std::vector<double> d_coeff = quinticPolynomialSolver(d_start, d_end, T);

  std::cout << "end: s=" << s_end[0] << ", s_dot=" << s_end[1] << ", s_dot_dot=" << s_end[2] << "\n"
            << "     d=" << d_end[0] << ", d_dot=" << d_end[1] << ", d_dot_dot=" << d_end[2] << std::endl;
  std::cout << "T=" << T << std::endl;

  for (size_t i = 0; i < num_waypoints_to_be_added; i++) {
    double s = map_->getValidS(getDistanceQuartic(s_coeff, i * DT));
    double s_dot = getVelocityQuartic(s_coeff, i * DT);
    double s_dot_dot = getAccelerationQuartic(s_coeff, i * DT);
    double d = getDistanceQuintic(d_coeff, i * DT);
    double d_dot = getVelocityQuintic(d_coeff, i * DT);
    double d_dot_dot = getAccelerationQuintic(d_coeff, i * DT);
    std::vector<double> xy = map_->getXY(s, d);

    Waypoint wp(s, d, s_dot, d_dot, s_dot_dot, d_dot_dot);
    wp.setX(xy[0]);
    wp.setY(xy[1]);
    trajectory->addWaypoint(wp);
  }
}

std::vector<double> Planner::quinticPolynomialSolver(const std::vector<double> &start,
                                                     const std::vector<double> &end,
                                                     const double T) const
{
  double si = start[0];
  double si_dot = start[1];
  double si_double_dot = start[2];
  double sf = end[0];
  double sf_dot = end[1];
  double sf_double_dot = end[2];
  double T2 = T * T;
  double T3 = T * T2;
  double T4 = T * T3;
  double T5 = T * T4;
  Eigen::MatrixXd lhs(3, 3);
  lhs << T3, T4, T5,
      3 * T2, 4 * T3, 5 * T4,
      6 * T, 12 * T2, 20 * T3;
  Eigen::VectorXd rhs(3);
  rhs << sf - (si + si_dot * T + 0.5 * si_double_dot * T * T),
      sf_dot - (si_dot + si_double_dot * T),
      sf_double_dot - si_double_dot;
  Eigen::VectorXd sol = lhs.colPivHouseholderQr().solve(rhs);

  return {si, si_dot, 0.5 * si_double_dot, sol(0), sol(1), sol(2)};
}

double Planner::getDistanceQuintic(const std::vector<double> &coeff, const double t) const
{
  double t2 = t * t;
  double t3 = t * t2;
  double t4 = t * t3;
  double t5 = t * t4;
  return coeff[0] + coeff[1] * t + coeff[2] * t2 + coeff[3] * t3 + coeff[4] * t4 + coeff[5] * t5;
}

double Planner::getVelocityQuintic(const std::vector<double> &coeff, const double t) const
{
  double t2 = t * t;
  double t3 = t * t2;
  double t4 = t * t3;
  return coeff[1] + 2 * coeff[2] * t +  3 * coeff[3] * t2 + 4 * coeff[4] * t3 + 5 * coeff[5] * t4;
}

double Planner::getAccelerationQuintic(const std::vector<double> &coeff, const double t) const
{
  double t2 = t * t;
  double t3 = t * t2;
  return 2 * coeff[2] +  6 * coeff[3] * t + 12 * coeff[4] * t2 + 20 * coeff[5] * t3;
}

std::vector<double> Planner::quarticPolynomialSolver(const std::vector<double> &start,
                                                     const std::vector<double> &end,
                                                     const double T) const
{
  double si = start[0];
  double si_dot = start[1];
  double si_double_dot = start[2];
  double sf_dot = end[1];
  double sf_double_dot = end[2];
  double T2 = T * T;
  double T3 = T * T2;
  Eigen::MatrixXd lhs(2, 2);
  lhs << 3 * T2, 4 * T3,
      6 * T, 12 * T2;
  Eigen::VectorXd rhs(2);
  rhs << sf_dot - (si_dot + si_double_dot * T),
      sf_double_dot - si_double_dot;
  Eigen::VectorXd sol = lhs.colPivHouseholderQr().solve(rhs);

  return {si, si_dot, 0.5 * si_double_dot, sol(0), sol(1)};
}

double Planner::getDistanceQuartic(const std::vector<double> &coeff, const double t) const
{
  double t2 = t * t;
  double t3 = t * t2;
  double t4 = t * t3;
  return coeff[0] + coeff[1] * t + coeff[2] * t2 + coeff[3] * t3 + coeff[4] * t4;
}

double Planner::getVelocityQuartic(const std::vector<double> &coeff, const double t) const
{
  double t2 = t * t;
  double t3 = t * t2;
  return coeff[1] + 2 * coeff[2] * t +  3 * coeff[3] * t2 + 4 * coeff[4] * t3;
}

double Planner::getAccelerationQuartic(const std::vector<double> &coeff, const double t) const
{
  double t2 = t * t;
  return 2 * coeff[2] +  6 * coeff[3] * t + 12 * coeff[4] * t2;
}
