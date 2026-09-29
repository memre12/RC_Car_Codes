// Small dense convex QP by ADMM (the OSQP iteration, Stellato et al. 2020), Eigen only:
//
//     min 1/2 z' P z + q' z   s.t.  l <= A z <= u
//
// P is n x n positive semidefinite, A is m x n. The KKT-like matrix
// P + sigma I + rho A'A is factorised once per rho (Cholesky, n = 2N ~ 40 here);
// every iteration costs two matrix-vector products and a back-substitution.
// Warm start: the previous solution and duals are kept between calls (receding horizon).
#pragma once

#include <Eigen/Dense>

namespace mpc_tc {

struct QpSettings {
  int max_iter = 400;
  double eps_abs = 1e-5;
  double eps_rel = 1e-4;
  double rho = 0.1;
  double sigma = 1e-6;
  double alpha = 1.6;      // over-relaxation
  int adapt_every = 25;    // rho adaptation (OSQP rule) every this many iterations
};

struct QpResult {
  int iterations = 0;
  bool converged = false;
  double primal_residual = 0.0, dual_residual = 0.0;
};

class DenseQp {
 public:
  explicit DenseQp(const QpSettings& s = QpSettings()) : set_(s) {}

  void resize(int n, int m) {
    if (n == x_.size() && m == z_.size()) return;
    x_ = Eigen::VectorXd::Zero(n);
    z_ = Eigen::VectorXd::Zero(m);
    y_ = Eigen::VectorXd::Zero(m);
    rho_ = set_.rho;
  }
  // receding horizon: x = [u_0 .. u_{K-1}] in blocks of `block`; drop u_0, repeat the last
  void shift_warm_start(int block);
  QpResult solve(const Eigen::MatrixXd& P, const Eigen::VectorXd& q, const Eigen::MatrixXd& A,
                 const Eigen::VectorXd& l, const Eigen::VectorXd& u);
  const Eigen::VectorXd& solution() const { return x_; }
  Eigen::VectorXd& mutable_solution() { return x_; }
  void reset_duals() { y_.setZero(); }

 private:
  QpSettings set_;
  Eigen::VectorXd x_, z_, y_;
  double rho_ = 0.1;
  Eigen::LLT<Eigen::MatrixXd> llt_;
};

inline QpResult DenseQp::solve(const Eigen::MatrixXd& P, const Eigen::VectorXd& q, const Eigen::MatrixXd& A,
                               const Eigen::VectorXd& l, const Eigen::VectorXd& u) {
  const int n = static_cast<int>(q.size()), m = static_cast<int>(l.size());
  resize(n, m);
  const Eigen::MatrixXd AtA = A.transpose() * A;
  auto factor = [&]() {
    Eigen::MatrixXd K = P + rho_ * AtA;
    K.diagonal().array() += set_.sigma;
    llt_.compute(K);
  };
  factor();
  // warm start: project the kept z into the bounds
  z_ = (A * x_).cwiseMax(l).cwiseMin(u);
  QpResult res;
  Eigen::VectorXd xt(n), zt(m), rhs(n), Ax(m), Px(n), Aty(n), zprev(m);
  for (int it = 1; it <= set_.max_iter; ++it) {
    rhs = set_.sigma * x_ - q + A.transpose() * (rho_ * z_ - y_);
    xt = llt_.solve(rhs);
    zt = A * xt;
    x_ = set_.alpha * xt + (1.0 - set_.alpha) * x_;
    const Eigen::VectorXd zr = set_.alpha * zt + (1.0 - set_.alpha) * z_;
    zprev = z_;
    z_ = (zr + y_ / rho_).cwiseMax(l).cwiseMin(u);
    y_ += rho_ * (zr - z_);
    // residuals (OSQP termination)
    Ax = A * x_;
    Px = P * x_;
    Aty = A.transpose() * y_;
    const double rp = (Ax - z_).lpNorm<Eigen::Infinity>();
    const double rd = (Px + q + Aty).lpNorm<Eigen::Infinity>();
    const double ep = set_.eps_abs + set_.eps_rel * std::max(Ax.lpNorm<Eigen::Infinity>(), z_.lpNorm<Eigen::Infinity>());
    const double ed = set_.eps_abs + set_.eps_rel * std::max({Px.lpNorm<Eigen::Infinity>(), Aty.lpNorm<Eigen::Infinity>(),
                                                               q.lpNorm<Eigen::Infinity>()});
    res.iterations = it;
    res.primal_residual = rp;
    res.dual_residual = rd;
    if (rp <= ep && rd <= ed) {
      res.converged = true;
      break;
    }
    if (set_.adapt_every > 0 && it % set_.adapt_every == 0) {
      const double np = rp / std::max({Ax.lpNorm<Eigen::Infinity>(), z_.lpNorm<Eigen::Infinity>(), 1e-12});
      const double nd = rd / std::max({Px.lpNorm<Eigen::Infinity>(), Aty.lpNorm<Eigen::Infinity>(),
                                       q.lpNorm<Eigen::Infinity>(), 1e-12});
      const double r_new = std::min(1e6, std::max(1e-6, rho_ * std::sqrt(np / std::max(nd, 1e-12))));
      if (r_new > 5.0 * rho_ || r_new < 0.2 * rho_) {
        rho_ = r_new;
        factor();
      }
    }
  }
  return res;
}

inline void DenseQp::shift_warm_start(int block) {
  const int K = static_cast<int>(x_.size()) / block;
  for (int k = 0; k + 1 < K; ++k) x_.segment(k * block, block) = x_.segment((k + 1) * block, block);
  y_.setZero();  // the constraint rows move with the horizon: restart the duals
}

}  // namespace mpc_tc
