#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "mpc_tracking_controller/dense_qp.hpp"
#include "mpc_tracking_controller/path_geometry.hpp"
#include "mpc_tracking_controller/tracking_mpc.hpp"

using namespace mpc_tc;

static void circle(double R, int n, std::vector<double>& x, std::vector<double>& y) {
  for (int i = 0; i < n; ++i) {
    const double a = 2.0 * M_PI * i / n;
    x.push_back(R * std::cos(a));
    y.push_back(R * std::sin(a));
  }
}

TEST(PathGeometry, CircleCurvatureAndProjection) {
  std::vector<double> x, y;
  circle(2.0, 400, x, y);  // counter-clockwise, closed
  PathGeometry g(x, y);
  EXPECT_TRUE(g.closed());
  EXPECT_NEAR(g.length(), 2.0 * M_PI * 2.0, 0.01);
  for (double s = 0.0; s < g.length(); s += 0.37) EXPECT_NEAR(g.kappa_at(s), 0.5, 2e-3);
  // point 0.1 m inside the circle (left of a CCW path) at angle 0, heading tangent (+y) + 0.2 rad
  const Projection p = g.project(1.9, 0.0, M_PI / 2 + 0.2);
  EXPECT_NEAR(p.e_lat, 0.1, 1e-3);
  EXPECT_NEAR(p.e_psi, 0.2, 5e-3);
}

TEST(PathGeometry, StraightLine) {
  std::vector<double> x, y;
  for (int i = 0; i <= 100; ++i) {
    x.push_back(0.1 * i);
    y.push_back(0.0);
  }
  PathGeometry g(x, y);
  EXPECT_FALSE(g.closed());
  EXPECT_NEAR(g.kappa_at(5.0), 0.0, 1e-9);
  const Projection p = g.project(3.0, -0.25, 0.1);
  EXPECT_NEAR(p.s, 3.0, 1e-9);
  EXPECT_NEAR(p.e_lat, -0.25, 1e-9);
}

TEST(DenseQp, MatchesUnconstrainedAndBoxSolutions) {
  std::mt19937 rng(3);
  std::normal_distribution<double> N(0.0, 1.0);
  const int n = 8;
  Eigen::MatrixXd M(n, n);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) M(i, j) = N(rng);
  const Eigen::MatrixXd P = M * M.transpose() + Eigen::MatrixXd::Identity(n, n);
  Eigen::VectorXd q(n);
  for (int i = 0; i < n; ++i) q[i] = N(rng);
  const Eigen::MatrixXd A = Eigen::MatrixXd::Identity(n, n);
  const Eigen::VectorXd x_star = -P.ldlt().solve(q);
  // loose bounds: the unconstrained optimum
  {
    QpSettings s;
    s.eps_abs = 1e-9;
    s.eps_rel = 1e-9;
    s.max_iter = 20000;
    DenseQp qp(s);
    const Eigen::VectorXd big = Eigen::VectorXd::Constant(n, 1e3);
    const QpResult r = qp.solve(P, q, A, -big, big);
    EXPECT_TRUE(r.converged);
    EXPECT_LT((qp.solution() - x_star).norm(), 1e-6);
  }
  // tight box: compare with projected gradient run to convergence
  {
    const Eigen::VectorXd lo = Eigen::VectorXd::Constant(n, -0.05), hi = Eigen::VectorXd::Constant(n, 0.05);
    Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
    const double step = 1.0 / P.eigenvalues().real().maxCoeff();
    for (int it = 0; it < 200000; ++it) x = (x - step * (P * x + q)).cwiseMax(lo).cwiseMin(hi);
    QpSettings s;
    s.eps_abs = 1e-9;
    s.eps_rel = 1e-9;
    s.max_iter = 20000;
    DenseQp qp(s);
    const QpResult r = qp.solve(P, q, A, lo, hi);
    EXPECT_TRUE(r.converged);
    EXPECT_LT((qp.solution() - x).norm(), 1e-5);
  }
}

TEST(TrackingController, SteeringMapInverse) {
  ControllerParams p;
  TrackingController c(p);
  for (double d = -0.32; d <= 0.32; d += 0.01) EXPECT_NEAR(c.eff_to_steer(c.steer_to_eff(d)), d, 1e-12);
  EXPECT_NEAR(c.steer_to_eff(0.32), 0.347, 1e-12);
  EXPECT_NEAR(c.steer_to_eff(-0.32), -0.314, 1e-12);
}

TEST(TrackingMpc, StraightLineOffsetConverges) {
  // the MPC model as the plant (no delays beyond the model's): a 0.3 m offset decays, within the bounds
  ModelParams m;
  MpcSettings s;
  s.speed_delay_steps = 0;
  std::vector<double> x, y;
  for (int i = 0; i <= 400; ++i) {
    x.push_back(0.1 * i);
    y.push_back(0.0);
  }
  PathGeometry g(x, y);
  TrackingMpc mpc(m, s);
  Eigen::Vector4d st(0.3, 0.0, 1.0, 0.0);
  double sp = 2.0, v_prev = 1.0, d_prev = 0.0;
  for (int k = 0; k < 60; ++k) {
    const MpcResult r = mpc.solve(st, sp, 1.0, g, v_prev, d_prev);
    ASSERT_TRUE(r.ok);
    EXPECT_LE(r.delta_eff, s.delta_hi + 1e-3);
    EXPECT_GE(r.delta_eff, s.delta_lo - 1e-3);
    EXPECT_LE(std::fabs(r.delta_eff - d_prev), s.delta_rate_max * s.dt + 1e-3);
    double ds = 0.0;
    st = mpc.step(st, r.v_cmd, r.delta_eff, 0.0, ds);
    sp += ds;
    v_prev = r.v_cmd;
    d_prev = r.delta_eff;
  }
  EXPECT_LT(std::fabs(st[0]), 0.01);
  EXPECT_LT(std::fabs(st[1]), 0.01);
  EXPECT_NEAR(st[2], 1.0 * m.speed_gain, 0.05);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
