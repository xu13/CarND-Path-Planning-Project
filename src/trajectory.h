#pragma once

#include "helper.h"

/**
 * @brief The Waypoint class.
 */
class Waypoint {
public:
  Waypoint() = default;

  Waypoint(const double s, const double d, const double s_dot, const double d_dot,
           const double s_dot_dot = 0.0, const double d_dot_dot = 0.0)
    : s_(s), d_(d), s_dot_(s_dot), d_dot_(d_dot),
      s_dot_dot_(s_dot_dot), d_dot_dot_(d_dot_dot)
  { }

  double getS() const { return s_; }
  double getSdot() const { return s_dot_; }
  double getSdotdot() const { return s_dot_dot_; }
  double getD() const { return d_; }
  double getDdot() const { return d_dot_; }
  double getDdotdot() const { return d_dot_dot_; }
  double getX() const { return x_; }
  double getY() const { return y_; }

  void setS(const double s) { s_ = s; }
  void setSdot(const double s_dot) { s_dot_ = s_dot; }
  void setSdotdot(const double s_dot_dot) { s_dot_dot_ = s_dot_dot; }
  void setD(const double d) { d_ = d; }
  void setDdot(const double d_dot) { d_dot_ = d_dot; }
  void setDdotdot(const double d_dot_dot) { d_dot_dot_ = d_dot_dot; }
  void setX(const double x) { x_ = x; }
  void setY(const double y) { y_ = y; }

  friend std::ostream& operator<<(std::ostream& out, const Waypoint& wp) {
    out << "Waypoint(x=" << wp.getX() << ", y=" << wp.getY()
        << ", s=" << wp.getS() << ", d=" << wp.getD() << ")";
    return out;
  }

private:
  double s_;
  double d_;
  double s_dot_;
  double d_dot_;
  double s_dot_dot_;
  double d_dot_dot_;
  double x_;
  double y_;
};

/**
 * @brief The Trajectory class.
 */
class Trajectory {
public:
  Trajectory() = default;

  size_t size() const { return waypoints_.size(); }

  void addWaypoint(const Waypoint& wp) { waypoints_.push_back(wp); }

  const std::vector<Waypoint>& getWaypoints() const { return waypoints_; }
  const Waypoint& getWaypoint(const size_t index) const { return waypoints_[index]; }
  const Waypoint& getLastWaypoint() const { return waypoints_.back(); }

  friend std::ostream& operator<<(std::ostream& out, const Trajectory& trajectory) {
    out << "Trajectory\n" << "{\n";
    for (auto& wp : trajectory.getWaypoints()) {
      out << wp;
    }
    out << "}";
    return out;
  }

private:
  std::vector<Waypoint> waypoints_;
};
