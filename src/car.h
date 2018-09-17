#pragma once

#include "helper.h"
#include "trajectory.h"

enum State {
  VELOCITY_KEEPING = 0,
  FOLLOWING,
  CHANGE_LANE_LEFT,
  CHANGE_LANE_RIGHT,
  NUM_OF_STATES
};

const std::string state_str[State::NUM_OF_STATES] = {"VELOCITY KEEPING",
                                                     "FOLLOWING",
                                                     "CHANGE LANE LEFT",
                                                     "CHANGE LANE RIGHT"};

class Car {
public:
  Car() = default;

  double getX() const { return x_; }
  double getY() const { return y_; }
  double getS() const { return s_; }
  double getD() const { return d_; }
  double getYaw() const { return yaw_; }
  double getSpeed() const { return speed_; }

  State getState() const { return state_; }
  void setState(State state) { state_ = state; }

//  std::vector<double> getPreviousPathX() const { return previous_path_x_; }
//  std::vector<double> getPreviousPathY() const { return previous_path_y_; }
//  size_t getPreviousPathSize() const { return previous_path_x_.size(); }
//  double getPreviousPathEndS() const { return end_path_s_; }
//  double getPreviousPathEndD() const { return end_path_d_; }

  Trajectory getPreviousTrajectory() const { return previous_trajectory_; }
  size_t getPreviousTrajectorySize() const { return previous_trajectory_.size(); }

  void setPreviousTrajectory(const Trajectory& trajectory) { previous_trajectory_ = trajectory; }

  void update(double x, double y, double s, double d, double yaw, double speed) {
    x_ = x;
    y_ = y;
    s_ = s;
    d_ = d;
    yaw_ = yaw;
    speed_ = speed;
//    previous_path_x_ = previous_path_x;
//    previous_path_y_ = previous_path_y;
//    end_path_s_ = end_path_s;
//    end_path_d_ = end_path_d;
  }

  friend std::ostream& operator<<(std::ostream& out, const Car& car) {
    out << "Car(x=" << car.getX() << ", y=" << car.getY()
        << ", s=" << car.getS() << ", d=" << car.getD()
        << ", yaw=" << car.getYaw() << ", speed=" << car.getSpeed() << ")";
    return out;
  }

private:
  double x_;
  double y_;
  double s_;
  double d_;
  double yaw_;
  double speed_;

  Trajectory previous_trajectory_;

  State state_ = State::VELOCITY_KEEPING;

//  // Previous path data given to the Planner
//  std::vector<double> previous_path_x_;
//  std::vector<double> previous_path_y_;

//  // Previous path's end s and d values
//  double end_path_s_;
//  double end_path_d_;
};
