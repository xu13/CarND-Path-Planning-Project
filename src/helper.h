#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"


// For converting back and forth between radians and degrees.
inline double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

/**
 * @brief Convert mph to m/s.
 * @param x
 * @return
 */
inline double mph2mps(double x) {
  return x / 2.23694;
}

/**
 * @brief Convert m/s to mph.
 * @param x
 * @return
 */
inline double mps2mph(double x) {
  return x * 2.23694;
}

/**
 * @brief Compute the distance between two points.
 * @param x1
 * @param y1
 * @param x2
 * @param y2
 * @return
 */
inline double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
