#pragma once

#include "helper.h"
#include "trajectory.h"

/**
 * @brief The State enum.
 */
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

/**
 * @brief The Car class.
 */
class Car {
public:
  Car(const int lane_id)
    : target_lane_id_(lane_id)
  {}

  double getX() const { return x_; }
  double getY() const { return y_; }
  double getS() const { return s_; }
  double getD() const { return d_; }
  double getYaw() const { return yaw_; }
  double getSpeed() const { return speed_; }

  /**
   * @brief Update vehicle information.
   * @param x
   * @param y
   * @param s
   * @param d
   * @param yaw
   * @param speed
   */
  void update(double x, double y, double s, double d, double yaw, double speed) {
    x_ = x;
    y_ = y;
    s_ = s;
    d_ = d;
    yaw_ = yaw;
    speed_ = speed;
  }

  int getTargetLaneId() const { return target_lane_id_; }
  void setTargetLaneId(const int lane_id) { target_lane_id_ = lane_id; }

  State getState() const { return state_; }
  void setState(State state) { state_ = state; }

  Trajectory getPreviousTrajectory() const { return previous_trajectory_; }
  size_t getPreviousTrajectorySize() const { return previous_trajectory_.size(); }
  void setPreviousTrajectory(const Trajectory& trajectory) { previous_trajectory_ = trajectory; }

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
  int target_lane_id_;
  State state_ = State::VELOCITY_KEEPING;
  Trajectory previous_trajectory_;
};
