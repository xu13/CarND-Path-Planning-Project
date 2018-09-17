#pragma once

#include "helper.h"

class Object {
public:
  Object(const int id, const double x, const double y, const double vx, const double vy,
         const double s, const double d, const double vs, const double vd)
    : id_(id), x_(x), y_(y), vx_(vx), vy_(vy), s_(s), d_(d), vs_(vs), vd_(vd)
  {}

  int getId() const { return id_; }
  double getX() const { return x_; }
  double getY() const { return y_; }
  double getS() const { return s_; }
  double getD() const { return d_; }
  double getXdot() const { return vx_; }
  double getYdot() const { return vy_; }
  double getSdot() const { return vs_; }
  double getDdot() const { return vd_; }
  double getSpeed() const { return std::hypot(vx_, vy_); }

  friend std::ostream& operator<<(std::ostream& out, const Object& car) {
    out << "Object(id=" << car.getId() << ", x=" << car.getX() << ", y=" << car.getY()
        << ", s=" << car.getS() << ", d=" << car.getD()
        << ", vx=" << car.getXdot() << ", vy=" << car.getYdot()
        << ", vs=" << car.getSdot() << ", vd=" << car.getDdot()
        << ", speed=" << car.getSpeed() << ")";
    return out;
  }

private:
  int id_;
  double x_;
  double y_;
  double vx_;
  double vy_;
  double s_;
  double d_;
  double vs_;
  double vd_;
};

