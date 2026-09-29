// Closed-loop check of the controller core WITHOUT ROS, on the car as identified
// (RC_Car_Sim car_like.json, fair_bags): the same controller code as the node.
//
//   mpc_closed_loop <path.csv> [speed_profile.csv] [--laps 5] [--noise 1] [--seed 1] [--series out.csv]
//
// Plant: kinematic bicycle (rear axle, L 0.28 m); steering command -> 50 ms dead time -> 40 ms lag
// -> identified map; speed command -> 0.18 s dead time -> 0.25 s lag, gain 1.005. Sensors: pose of the
// point 0.17 m ahead at 8.44 Hz, position 0.239 s / heading 0.200 s old, heading 0.045 rad biased,
// white noise 0.0127 m lateral / 0.0047 rad heading (--noise 0: none); IMU 197 Hz (0.049 rad/s
// noise), VESC speed 50 Hz; command published 14 ms after the pose (Jetson). Not modelled: the
// slow part of the localisation noise, the yaw disturbance, jitter (Gazebo car-like mode has them).
// Reports tracking metrics against the TRUE position and the wall-clock time of each control step.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "mpc_tracking_controller/tracking_mpc.hpp"

using namespace mpc_tc;

static bool read_csv2(const std::string& fn, std::vector<double>& a, std::vector<double>& b) {
  std::ifstream f(fn);
  if (!f) return false;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#' || !(std::isdigit(line[0]) || line[0] == '-' || line[0] == '.')) continue;
    std::stringstream ss(line);
    std::string c1, c2;
    std::getline(ss, c1, ',');
    std::getline(ss, c2, ',');
    a.push_back(std::stod(c1));
    b.push_back(std::stod(c2));
  }
  return !a.empty();
}

struct Delayed {  // dead time + first-order lag, integrated with the plant step
  Delayed(double d, double ta, double g) : dead(d), tau(ta), gain(g) {}
  double dead, tau, gain, x = 0.0;
  std::deque<std::pair<double, double>> q;  // (effective time, command)
  double u = 0.0;
  void cmd(double t, double c) { q.emplace_back(t + dead, c); }
  void step(double t, double h) {
    while (!q.empty() && q.front().first <= t) {
      u = q.front().second;
      q.pop_front();
    }
    x = gain * u + (x - gain * u) * std::exp(-h / tau);
  }
};

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s path.csv [speed_profile.csv] [--laps N] [--noise 0|1] [--seed S] [--series out.csv]\n", argv[0]);
    return 2;
  }
  std::vector<double> px, py, ps, pv;
  if (!read_csv2(argv[1], px, py)) {
    std::fprintf(stderr, "cannot read %s\n", argv[1]);
    return 2;
  }
  double laps = 5.0, v_const = 0.9;
  int noise = 1;
  unsigned seed = 1;
  std::string series;
  bool have_profile = false;
  int horizon = 20;
  for (int i = 2; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--laps") && i + 1 < argc) laps = std::stod(argv[++i]);
    else if (!std::strcmp(argv[i], "--noise") && i + 1 < argc) noise = std::stoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--seed") && i + 1 < argc) seed = std::stoul(argv[++i]);
    else if (!std::strcmp(argv[i], "--series") && i + 1 < argc) series = argv[++i];
    else if (!std::strcmp(argv[i], "--speed") && i + 1 < argc) v_const = std::stod(argv[++i]);
    else if (!std::strcmp(argv[i], "--horizon") && i + 1 < argc) horizon = std::stoi(argv[++i]);
    else have_profile = read_csv2(argv[i], ps, pv);
  }
  ControllerParams P;  // defaults = the identified car
  P.mpc.horizon = horizon;
  {
    const TrackingController tmp(P);
    P.mpc.delta_lo = tmp.steer_to_eff(-P.delta_cmd_max);
    P.mpc.delta_hi = tmp.steer_to_eff(P.delta_cmd_max);
  }
  P.mpc.speed_delay_steps = static_cast<int>(std::lround((P.speed_dead_time - P.steer_dead_time) / P.mpc.dt));
  TrackingController ctrl(P);
  ctrl.set_path(px, py);
  PathGeometry truth(px, py);  // evaluation (separate projection state)
  const double L = 0.28, a = 0.17, bias = 0.045;
  Delayed steer(0.05, 0.04, 1.0), speed(0.18, 0.25, 1.005);
  std::mt19937 rng(seed);
  std::normal_distribution<double> N01(0.0, 1.0);

  // start on the path at s = 5 m, at rest
  double x, y;
  truth.point_at(5.0, x, y);
  double psi = truth.psi_at(5.0);
  x -= a * std::cos(psi);
  y -= a * std::sin(psi);
  struct Hist { double t, x, y, psi; };
  std::deque<Hist> hist;
  const double h = 0.001, T_pose = 1.0 / 8.44, T_imu = 1.0 / 197.0, T_vesc = 0.02, t_compute = 0.014;
  double t = 0.0, next_pose = 0.5, next_imu = 0.0, next_vesc = 0.0, s_trav = 0.0;
  double pending_t = -1.0, pend_v = 0.0, pend_d = 0.0;
  const double t_end = 600.0;
  std::vector<double> el, el_st, el_cu, d_st, step_ms;
  std::ofstream ser;
  if (!series.empty()) {
    ser.open(series);
    ser << "t,s,e_lat,e_psi,v,v_ref,delta_cmd,kappa\n";
  }
  double last_s = truth.project(x + a * std::cos(psi), y + a * std::sin(psi), psi).s;
  double v_ref_now = v_const;
  double last_dcmd = 0.0;
  while (t < t_end && s_trav < laps * truth.length()) {
    // plant
    steer.step(t, h);
    speed.step(t, h);
    const double delta = ctrl.steer_to_eff(steer.x);  // lagged servo command -> identified map
    const double v = speed.x;
    const double r = v * std::tan(delta) / L;
    x += h * v * std::cos(psi + 0.5 * h * r);
    y += h * v * std::sin(psi + 0.5 * h * r);
    psi += h * r;
    t += h;
    hist.push_back({t, x, y, psi});
    while (hist.size() > 2 && hist.front().t < t - 1.0) hist.pop_front();
    // sensors
    if (t >= next_imu) {
      ctrl.on_imu(t, r + (noise ? 0.049 * N01(rng) : 0.0));
      next_imu += T_imu;
    }
    if (t >= next_vesc) {
      ctrl.on_speed(t, v);
      next_vesc += T_vesc;
    }
    // command publication (compute delay after the pose)
    if (pending_t >= 0.0 && t >= pending_t) {
      ctrl.on_command_sent(t, pend_v, pend_d);
      steer.cmd(t, pend_d);
      speed.cmd(t, pend_v);
      last_dcmd = pend_d;
      pending_t = -1.0;
    }
    if (t >= next_pose) {
      next_pose += T_pose;
      auto at = [&](double tq) {
        for (auto it = hist.rbegin(); it != hist.rend(); ++it)
          if (it->t <= tq) return *it;
        return hist.front();
      };
      const Hist hp = at(t - P.pose_latency_position), hh = at(t - P.pose_latency_heading);
      double mx = hp.x + a * std::cos(hp.psi), my = hp.y + a * std::sin(hp.psi);
      double myaw = hh.psi - bias;
      if (noise) {
        const double n = 0.0127 * N01(rng);
        mx += -std::sin(hp.psi) * n;
        my += std::cos(hp.psi) * n;
        myaw += 0.0047 * N01(rng);
      }
      // reference speed
      const Projection tp = truth.project(x + a * std::cos(psi), y + a * std::sin(psi), psi);
      if (have_profile) {
        double sq = std::fmod(tp.s, truth.length());
        size_t i = 0;
        while (i + 1 < ps.size() && ps[i + 1] < sq) ++i;
        v_ref_now = pv[std::min(i, pv.size() - 1)];
      }
      const auto w0 = std::chrono::steady_clock::now();
      const ControlOutput o = ctrl.on_pose(t, mx, my, myaw, v_ref_now);
      step_ms.push_back(1e3 * std::chrono::duration<double>(std::chrono::steady_clock::now() - w0).count());
      if (o.status != Status::kOk) {
        std::fprintf(stderr, "t=%.2f status %d (e_lat %.2f): stop\n", t, static_cast<int>(o.status), o.meas.e_lat);
        return 3;
      }
      pending_t = t + t_compute;
      pend_v = o.v_cmd;
      pend_d = o.delta_cmd;
      // evaluation on the true state (after 10 s)
      double ds = tp.s - last_s;
      if (ds < -0.5 * truth.length()) ds += truth.length();
      if (ds > 0.5 * truth.length()) ds -= truth.length();
      s_trav += ds;
      last_s = tp.s;
      double kmax = 0.0;
      for (double q = -0.5; q <= 0.5; q += 0.1) kmax = std::max(kmax, std::fabs(truth.kappa_at(tp.s + q)));
      if (t > 10.0) {
        el.push_back(tp.e_lat);
        if (kmax < 0.15) {
          el_st.push_back(tp.e_lat);
          d_st.push_back(last_dcmd);
        }
        if (std::fabs(tp.kappa) > 0.6) el_cu.push_back(tp.e_lat);
      }
      if (ser) ser << t << "," << tp.s << "," << tp.e_lat << "," << tp.e_psi << "," << v << "," << v_ref_now << "," << o.delta_cmd << "," << tp.kappa << "\n";
    }
  }
  auto rms = [](const std::vector<double>& v) {
    double s = 0.0;
    for (double e : v) s += e * e;
    return v.empty() ? NAN : std::sqrt(s / v.size());
  };
  auto stdv = [](const std::vector<double>& v) {
    double m = 0.0, s = 0.0;
    for (double e : v) m += e;
    m /= std::max<size_t>(v.size(), 1);
    for (double e : v) s += (e - m) * (e - m);
    return v.empty() ? NAN : std::sqrt(s / v.size());
  };
  std::vector<double> sm = step_ms;
  std::sort(sm.begin(), sm.end());
  std::printf("laps %.2f in %.1f s | RMS e_lat all %.3f straights %.3f curves %.3f m | steering std straights %.3f rad | "
              "step %.3f ms median, %.3f p99, %.3f max\n",
              s_trav / truth.length(), t, rms(el), rms(el_st), rms(el_cu), stdv(d_st), sm[sm.size() / 2],
              sm[static_cast<size_t>(0.99 * (sm.size() - 1))], sm.back());
  return 0;
}
