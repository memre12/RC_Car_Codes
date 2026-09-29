#include "mpc_tracking_controller/path_geometry.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace mpc_tc {

namespace {
constexpr double kClosedTolerance = 0.5;  // m, as path_geometry.py

// Gaussian smoothing over grid samples (sigma in samples), path_geometry.py::_gaussian_smooth.
std::vector<double> gaussian_smooth(const std::vector<double>& v, double sigma, bool closed) {
  const int half = static_cast<int>(std::ceil(3.0 * sigma));
  if (half < 1) return v;
  std::vector<double> k(2 * half + 1);
  double sum = 0.0;
  for (int j = -half; j <= half; ++j) sum += (k[j + half] = std::exp(-0.5 * (j / sigma) * (j / sigma)));
  for (double& w : k) w /= sum;
  const int n = static_cast<int>(v.size());
  std::vector<double> out;
  if (closed) {
    // v.back() duplicates v.front() (+ total turn): smooth the open period, pad circularly
    const int m = n - 1;
    const double total = v.back() - v.front();
    out.resize(m + 1);
    for (int i = 0; i < m; ++i) {
      double acc = 0.0;
      for (int j = -half; j <= half; ++j) {
        const int idx = i + j;
        const int wraps = static_cast<int>(std::floor(static_cast<double>(idx) / m));
        acc += k[j + half] * (v[((idx % m) + m) % m] + wraps * total);
      }
      out[i] = acc;
    }
    out[m] = out[0] + total;
    return out;
  }
  // open: odd reflection about the end points
  auto at = [&](int idx) {
    if (idx < 0) return 2.0 * v[0] - v[-idx];
    if (idx >= n) return 2.0 * v[n - 1] - v[2 * (n - 1) - idx];
    return v[idx];
  };
  out.resize(n);
  for (int i = 0; i < n; ++i) {
    double acc = 0.0;
    for (int j = -half; j <= half; ++j) acc += k[j + half] * at(i + j);
    out[i] = acc;
  }
  return out;
}
}  // namespace

PathGeometry::PathGeometry(const std::vector<double>& xs_in, const std::vector<double>& ys_in, double resample_ds,
                           double smoothing_length) {
  if (xs_in.size() != ys_in.size()) throw std::invalid_argument("path: x/y size mismatch");
  std::vector<double> xs, ys;
  for (size_t i = 0; i < xs_in.size(); ++i) {
    if (i > 0 && std::hypot(xs_in[i] - xs.back(), ys_in[i] - ys.back()) <= 1e-9) continue;  // duplicates
    xs.push_back(xs_in[i]);
    ys.push_back(ys_in[i]);
  }
  if (xs.size() < 3) throw std::invalid_argument("a path needs at least 3 distinct points");
  const double gap = std::hypot(xs.back() - xs.front(), ys.back() - ys.front());
  closed_ = gap <= kClosedTolerance;
  if (closed_ && gap > 1e-9) {
    xs.push_back(xs.front());
    ys.push_back(ys.front());
  }
  std::vector<double> s_raw(xs.size(), 0.0);
  for (size_t i = 1; i < xs.size(); ++i) s_raw[i] = s_raw[i - 1] + std::hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]);
  length_ = s_raw.back();

  const int n = std::max(static_cast<int>(std::ceil(length_ / resample_ds)) + 1, 4);
  s_.resize(n);
  x_.resize(n);
  y_.resize(n);
  size_t j = 0;
  for (int i = 0; i < n; ++i) {
    const double s = length_ * i / (n - 1);
    s_[i] = s;
    while (j + 2 < s_raw.size() && s_raw[j + 1] < s) ++j;
    const double w = (s - s_raw[j]) / std::max(s_raw[j + 1] - s_raw[j], 1e-12);
    const double wc = std::min(1.0, std::max(0.0, w));
    x_[i] = xs[j] + wc * (xs[j + 1] - xs[j]);
    y_[i] = ys[j] + wc * (ys[j + 1] - ys[j]);
  }
  ds_ = s_[1] - s_[0];

  // tangent heading (central differences), unwrapped
  std::vector<double> dx(n), dy(n), psi(n);
  for (int i = 0; i < n; ++i) {
    if (closed_) {
      if (i == 0 || i == n - 1) {
        dx[i] = x_[1] - x_[n - 2];
        dy[i] = y_[1] - y_[n - 2];
      } else {
        dx[i] = x_[i + 1] - x_[i - 1];
        dy[i] = y_[i + 1] - y_[i - 1];
      }
    } else {  // numpy.gradient: central inside, one-sided at the ends
      const int a = std::max(i - 1, 0), b = std::min(i + 1, n - 1);
      dx[i] = (x_[b] - x_[a]) / (b - a);
      dy[i] = (y_[b] - y_[a]) / (b - a);
    }
    psi[i] = std::atan2(dy[i], dx[i]);
    if (i > 0) psi[i] = psi[i - 1] + wrap_angle(psi[i] - psi[i - 1]);
  }
  psi_un_ = gaussian_smooth(psi, std::max(smoothing_length / ds_, 1e-6), closed_);

  kappa_.resize(n);
  if (closed_) {
    const double total = psi_un_[n - 1] - psi_un_[0];
    const int m = n - 1;
    for (int i = 0; i < m; ++i) {
      const double nxt = (i + 1 < m) ? psi_un_[i + 1] : psi_un_[0] + total;
      const double prv = (i - 1 >= 0) ? psi_un_[i - 1] : psi_un_[m - 1] - total;
      kappa_[i] = (nxt - prv) / (2.0 * ds_);
    }
    kappa_[m] = kappa_[0];
  } else {
    for (int i = 0; i < n; ++i) {
      if (i == 0) kappa_[i] = (psi_un_[1] - psi_un_[0]) / ds_;
      else if (i == n - 1) kappa_[i] = (psi_un_[n - 1] - psi_un_[n - 2]) / ds_;
      else kappa_[i] = (psi_un_[i + 1] - psi_un_[i - 1]) / (2.0 * ds_);
    }
  }
}

double PathGeometry::norm_s(double s) const {
  if (closed_) {
    s = std::fmod(s, length_);
    return s < 0.0 ? s + length_ : s;
  }
  return std::min(std::max(s, 0.0), length_);
}

double PathGeometry::interp(const std::vector<double>& v, double s) const {
  s = norm_s(s);
  const double f = s / ds_;
  int i = static_cast<int>(f);
  if (i >= static_cast<int>(v.size()) - 1) return v.back();
  const double w = f - i;
  return v[i] + w * (v[i + 1] - v[i]);
}

void PathGeometry::best_segment(double px, double py, const std::vector<int>& idx, double& d2, int& bi,
                                double& bt) const {
  d2 = std::numeric_limits<double>::infinity();
  for (int i : idx) {
    const double ax = x_[i], ay = y_[i], sx = x_[i + 1] - ax, sy = y_[i + 1] - ay;
    const double l2 = std::max(sx * sx + sy * sy, 1e-12);
    const double t = std::min(1.0, std::max(0.0, ((px - ax) * sx + (py - ay) * sy) / l2));
    const double qx = ax + t * sx - px, qy = ay + t * sy - py;
    const double d = qx * qx + qy * qy;
    if (d < d2) {
      d2 = d;
      bi = i;
      bt = t;
    }
  }
}

Projection PathGeometry::project(double px, double py, double yaw, double window, double relocalize_distance) {
  const int n = static_cast<int>(s_.size());
  double d2 = std::numeric_limits<double>::infinity(), t = 0.0;
  int i = 0;
  bool found = false;
  if (last_idx_ >= 0) {
    const int w = static_cast<int>(std::ceil(window / ds_));
    std::vector<int> idx;
    idx.reserve(2 * w + 1);
    for (int k = last_idx_ - w; k <= last_idx_ + w; ++k) {
      if (closed_) idx.push_back(((k % (n - 1)) + (n - 1)) % (n - 1));
      else if (k >= 0 && k < n - 1) idx.push_back(k);
    }
    best_segment(px, py, idx, d2, i, t);
    found = d2 <= relocalize_distance * relocalize_distance;
  }
  if (!found) {
    std::vector<int> aligned, all;
    for (int k = 0; k < n - 1; ++k) {
      all.push_back(k);
      const double h = std::atan2(y_[k + 1] - y_[k], x_[k + 1] - x_[k]);
      if (std::fabs(wrap_angle(yaw - h)) < 0.5 * M_PI) aligned.push_back(k);
    }
    best_segment(px, py, aligned.empty() ? all : aligned, d2, i, t);
  }
  last_idx_ = i;
  Projection p;
  p.s = s_[i] + t * ds_;
  p.x = x_[i] + t * (x_[i + 1] - x_[i]);
  p.y = y_[i] + t * (y_[i + 1] - y_[i]);
  p.psi = psi_at(p.s);
  p.kappa = kappa_at(p.s);
  p.e_lat = -std::sin(p.psi) * (px - p.x) + std::cos(p.psi) * (py - p.y);
  p.e_psi = wrap_angle(yaw - p.psi);
  return p;
}

}  // namespace mpc_tc
