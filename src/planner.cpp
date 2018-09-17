#include "planner.h"

Planner::Planner(Map* m, int id)
{
  map_ = m;
  lane_id_ = id;
}

void Planner::plan(Car& car, const std::unordered_map<int, Object>& objects, const size_t leftover_size,
                   std::vector<double>* next_x_vals, std::vector<double>* next_y_vals)
{
  std::cout << "Current car state: " << car << std::endl;

  Trajectory trajectory;

  // Find the start waypoint for the next plan
  double start_s = car.getS();
  double start_s_dot = 0.0;
  double start_s_dot_dot = 0.0;
  double start_d = car.getD();
  double start_d_dot = 0.0;
  double start_d_dot_dot = 0.0;

  // If we already have previous plan, use the last point of that plan as a starting point for the new plan
  size_t num_waypoints_to_be_added = N;
  if (leftover_size > 0) {
    size_t num_waypoints_consumed = N - leftover_size;

    const Waypoint& wp = car.getPreviousTrajectory().getWaypoint(LOOKAHEAD_SIZE + num_waypoints_consumed);
    start_s = wp.getS();
    start_s_dot = wp.getSdot();
    start_s_dot_dot = wp.getSdotdot();
    start_d = wp.getD();
    start_d_dot = wp.getDdot();
    start_d_dot_dot = wp.getDdotdot();

    // Add the lookahead waypoints from the previous plan to the new plan
    for (size_t i = num_waypoints_consumed; i < num_waypoints_consumed + LOOKAHEAD_SIZE; i++) {
      const Waypoint& wp = car.getPreviousTrajectory().getWaypoint(i);
      trajectory.addWaypoint(wp);
    }
    num_waypoints_to_be_added = N - LOOKAHEAD_SIZE;
  }

  // Loop through all objects (other cars)
  std::pair<int, double> front_car = std::make_pair(-1, std::numeric_limits<double>::infinity());
  std::pair<int, double> front_left_car = std::make_pair(-1, std::numeric_limits<double>::infinity());
  std::pair<int, double> front_right_car = std::make_pair(-1, std::numeric_limits<double>::infinity());
  std::pair<int, double> rear_left_car = std::make_pair(-1, -std::numeric_limits<double>::infinity());
  std::pair<int, double> rear_right_car = std::make_pair(-1, -std::numeric_limits<double>::infinity());

  const int car_lane_id = map_->getLaneId(start_d);
  for (auto& p : objects) {
    const int id = p.first;
    const Object& obj = p.second;

    // Lane id of the object
    const int obj_lane_id = map_->getLaneId(obj.getD());
    // This is the s position of the object when the ego-car finishes last plan (projection ahead)
    double obj_s = std::fmod(obj.getS() + LOOKAHEAD_SIZE * obj.getSdot() * DT, map_->getMaxS());
    double ds = obj_s - start_s;
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

  // Starting points
  std::vector<double> s_start = {start_s, start_s_dot, start_s_dot_dot};
  std::cout << "start: s=" << start_s << ", s_dot=" << start_s_dot << ", s_dot_dot=" << start_s_dot_dot << std::endl;
  std::vector<double> d_start = {start_d, start_d_dot, start_d_dot_dot};
  std::cout << "start: d=" << start_d << ", d_dot=" << start_d_dot << ", d_dot_dot=" << start_d_dot_dot << std::endl;

  // Compute trajectory
  switch (car.getState()) {
    case State::VELOCITY_KEEPING: {
        computeVelocityKeepingTrajectory(s_start, d_start,
                                         num_waypoints_to_be_added, &trajectory);
        const double FRONT_SAFETY_DISTANCE = 30.0;
        if (front_car.second < FRONT_SAFETY_DISTANCE) {
          car.setState(State::FOLLOWING);
        }
        break;
      }
    case State::FOLLOWING: {
        computeFollowingTrajectory(s_start, d_start,
                                   objects.at(front_car.first), front_car.second,
                                   num_waypoints_to_be_added, &trajectory);
        const double FRONT_SAFETY_DISTANCE = 40.0;
        const double REAR_SAFETY_DISTANCE = -10.0;
        if (front_left_car.second > FRONT_SAFETY_DISTANCE &&
            rear_left_car.second < REAR_SAFETY_DISTANCE &&
            front_right_car.second > FRONT_SAFETY_DISTANCE &&
            rear_right_car.second < REAR_SAFETY_DISTANCE &&
            lane_id_ == 1) {
          if (front_left_car.second >= front_right_car.second) {
            car.setState(State::CHANGE_LANE_LEFT);
          } else {
            car.setState(State::CHANGE_LANE_RIGHT);
          }
        } else if (front_left_car.second > FRONT_SAFETY_DISTANCE &&
                   rear_left_car.second < REAR_SAFETY_DISTANCE &&
                   lane_id_ != 0) {
          car.setState(State::CHANGE_LANE_LEFT);
        } else if (front_right_car.second > FRONT_SAFETY_DISTANCE  &&
                   rear_right_car.second < REAR_SAFETY_DISTANCE  &&
                   lane_id_ != 2) {
          car.setState(State::CHANGE_LANE_RIGHT);
        }
        break;
      }
    case State::CHANGE_LANE_LEFT: {
        computeChangingLaneTrajectory(s_start, d_start,
                                      LaneChangeDir::LEFT, num_waypoints_to_be_added, &trajectory);
        double dd = map_->getCenterD(lane_id_ + LaneChangeDir::LEFT) - start_d;
        if (std::abs(dd) < 0.8) {
          lane_id_ += LaneChangeDir::LEFT;
          car.setState(State::VELOCITY_KEEPING);
        }
        break;
      }
    case State::CHANGE_LANE_RIGHT: {
        computeChangingLaneTrajectory(s_start, d_start,
                                      LaneChangeDir::RIGHT, num_waypoints_to_be_added, &trajectory);
        double dd = map_->getCenterD(lane_id_ + LaneChangeDir::RIGHT) - start_d;
        if (std::abs(dd) < 0.8) {
          lane_id_ += LaneChangeDir::RIGHT;
          car.setState(State::VELOCITY_KEEPING);
        }
        break;
      }
    default:
      break;
  }

  // Save the trajectory for next cycle use
  car.setPreviousTrajectory(trajectory);

  for (const Waypoint& wp : trajectory.getWaypoints()) {
    next_x_vals->push_back(wp.getX());
    next_y_vals->push_back(wp.getY());
  }

  std::cout << "Previous plan remaining size: " << leftover_size << std::endl;
  std::cout << "New plan size: " << num_waypoints_to_be_added << std::endl;
  std::cout << "Current state: " << state_str[car.getState()] << std::endl;
  std::cout << "Current lane id: " << map_->getLaneId(start_d) << std::endl;
  std::cout << "Current plan last waypoint: " << trajectory.getLastWaypoint() << std::endl;
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
  std::cout << "end: s=" << s_end[0] << ", s_dot=" << s_end[1] << ", s_dot_dot=" << s_end[2] << std::endl;
  std::vector<double> s_coeff = quarticPolynomialSolver(s_start, s_end, T);

  std::vector<double> d_end = {map_->getCenterD(lane_id_), 0.0, 0.0};
  std::cout << "end: d=" << d_end[0] << ", d_dot=" << d_end[1] << ", d_dot_dot=" << d_end[2] << std::endl;
  std::vector<double> d_coeff = quinticPolynomialSolver(d_start, d_end, T);
  std::cout << "T: " << T << std::endl;

  for (size_t i = 0; i < num_waypoints_to_be_added; i++) {
    double s = getDistanceQuartic(s_coeff, i * DT);
    s = std::fmod(s, map_->getMaxS());
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
                                         const Object& front_car, double min_ds,
                                         size_t num_waypoints_to_be_added,
                                         Trajectory* trajectory) const
{
  std::cout << "Front car: " << front_car << std::endl;
//  double end_s = s_start[0] + min_ds;
  double end_s_dot = front_car.getSdot();
//  const double T = 30.0 / SPEED_LIMIT;
  const double T = 3;
  double end_s = s_start[0] + min_ds + front_car.getSdot() * T - (10 + 2 * front_car.getSdot());

  std::vector<double> s_end = {end_s, end_s_dot, 0.0};
  std::cout << "end: s=" << s_end[0] << ", s_dot=" << s_end[1] << ", s_dot_dot=" << s_end[2] << std::endl;
  std::vector<double> s_coeff = quinticPolynomialSolver(s_start, s_end, T);

  std::vector<double> d_end = {map_->getCenterD(lane_id_), 0.0, 0.0};
  std::cout << "end: d=" << d_end[0] << ", d_dot=" << d_end[1] << ", d_dot_dot=" << d_end[2] << std::endl;
  std::vector<double> d_coeff = quinticPolynomialSolver(d_start, d_end, T);
  std::cout << "T: " << T << std::endl;

  for (size_t i = 0; i < num_waypoints_to_be_added; i++) {
    double s = getDistanceQuintic(s_coeff, i * DT);
    s = std::fmod(s, map_->getMaxS());
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
                                            LaneChangeDir dir,
                                            size_t num_waypoints_to_be_added,
                                            Trajectory* trajectory) const
{
//  double ref_v = SPEED_LIMIT;
//  if (s_start[1] < SPEED_LIMIT) {
//    ref_v = s_start[1] + 6.0;
//  }
  double ref_v = s_start[1];
  const double T = 3;

  std::vector<double> s_end = {-1, ref_v, 0};
  std::cout << "end: s=" << s_end[0] << ", s_dot=" << s_end[1] << ", s_dot_dot=" << s_end[2] << std::endl;
  std::vector<double> s_coeff = quarticPolynomialSolver(s_start, s_end, T);

  int target_lane_id = lane_id_ + dir;
  std::vector<double> d_end = {map_->getCenterD(target_lane_id), 0, 0};
  std::cout << "end: d=" << d_end[0] << ", d_dot=" << d_end[1] << ", d_dot_dot=" << d_end[2] << std::endl;
  std::vector<double> d_coeff = quinticPolynomialSolver(d_start, d_end, T);
  std::cout << "T: " << T << std::endl;

  for (size_t i = 0; i < num_waypoints_to_be_added; i++) {
    double s = getDistanceQuartic(s_coeff, i * DT);
    s = std::fmod(s, map_->getMaxS());
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
  //  double sf = end[0];
  double sf_dot = end[1];
  double sf_double_dot = end[2];
  double T2 = T * T;
  double T3 = T * T2;
  //  double T4 = T * T3;
  //  double T5 = T * T4;
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
