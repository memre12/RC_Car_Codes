// Arc-length parameterised path geometry: a C++ port of
// RC_Car_Codes/src/integral_rl_controller/integral_rl_controller/path_geometry.py,
// so that the MPC and the IRL controller compute e_lat, e_psi and kappa identically.
//
// * uniform arc-length resampling (0.05 m);
// * heading from central differences, Gaussian-smoothed over a fixed ARC LENGTH
//   (0.4 m), curvature = d psi / d s of the smoothed heading;
// * continuous, LOCAL projection (+-window around the previous projection), with a
//   heading-aligned global search on the first call or when the local result is far.
#pragma once

#include <cmath>
#include <vector>

namespace mpc_tc {

inline double wrap_angle(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a < 0.0) a += 2.0 * M_PI;
  return a - M_PI;
}

struct Projection {
  double s = 0.0;      // arc length of the projection point [m]
  double x = 0.0, y = 0.0;
  double psi = 0.0;    // path heading [rad]
  double kappa = 0.0;  // path curvature [1/m]
  double e_lat = 0.0;  // left of the path positive [m]
  double e_psi = 0.0;  // yaw - psi, wrapped [rad]
};

class PathGeometry {
 public:
  PathGeometry(const std::vector<double>& xs, const std::vector<double>& ys, double resample_ds = 0.05,
               double smoothing_length = 0.4);

  double length() const { return length_; }
  bool closed() const { return closed_; }
  double kappa_at(double s) const { return interp(kappa_, s); }
  double psi_at(double s) const { return wrap_angle(interp(psi_un_, s)); }
  void point_at(double s, double& x, double& y) const {
    x = interp(x_, s);
    y = interp(y_, s);
  }
  double norm_s(double s) const;
  Projection project(double px, double py, double yaw, double window = 0.6, double relocalize_distance = 1.0);
  void reset_tracking() { last_idx_ = -1; }

 private:
  double interp(const std::vector<double>& v, double s) const;
  void best_segment(double px, double py, const std::vector<int>& idx, double& d2, int& i, double& t) const;

  std::vector<double> s_, x_, y_, psi_un_, kappa_;
  double ds_ = 0.05, length_ = 0.0;
  bool closed_ = false;
  int last_idx_ = -1;
};

}  // namespace mpc_tc
