#pragma once
#include <Eigen/Dense>
#include <deque>

// 标准卡尔曼滤波器
struct StandardKF {
  double dt;
  Eigen::MatrixXd A, C;
  Eigen::MatrixXd Qt, Rt;
  Eigen::MatrixXd Sigma;
  Eigen::VectorXd x;

  StandardKF(double _dt) : dt(_dt) {
    A.setIdentity(6, 6);
    Sigma.setZero(6, 6);
    C.setIdentity(6, 6);
    A(0, 3) = dt;
    A(1, 4) = dt;
    A(2, 5) = dt;
    Qt.setIdentity(6, 6);
    Rt.setIdentity(6, 6);
    // 固定过程/观测噪声协方差
    Qt *= 0.1;
    Rt.topLeftCorner(3,3) *= 0.09;
    Rt.bottomRightCorner(3,3) *= 0.4;
    x.setZero(6);
  }
  void reset(const Eigen::Vector3d& z) {
    x.head(3) = z;
    x.tail(3).setZero();
    Sigma.setZero();
  }
  void predict() {
    x = A * x;
    Sigma = A * Sigma * A.transpose() + Qt;
  }
  void update(const Eigen::Vector3d& z_pos, const Eigen::Vector3d& z_vel) {
    Eigen::VectorXd z(6);
    z << z_pos, z_vel;
    Eigen::MatrixXd K = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    x = x + K * (z - C * x);
    Sigma = Sigma - K * C * Sigma;
  }
  Eigen::Vector3d pos() const { return x.head(3); }
  Eigen::Vector3d vel() const { return x.tail(3); }
};

// 基于新息序列协方差自适应的卡尔曼滤波器
struct AdaptiveKF {
  double dt;
  Eigen::MatrixXd A, C;
  Eigen::MatrixXd Qt, Rt;
  Eigen::MatrixXd Sigma;
  Eigen::VectorXd x;
  std::deque<Eigen::VectorXd> innovation_list;
  int window_size;

  AdaptiveKF(double _dt, int _window_size = 10) : dt(_dt), window_size(_window_size) {
    A.setIdentity(6, 6);
    Sigma.setZero(6, 6);
    C.setIdentity(6, 6);
    A(0, 3) = dt;
    A(1, 4) = dt;
    A(2, 5) = dt;
    Qt.setIdentity(6, 6);
    Rt.setIdentity(6, 6);
    Qt *= 0.1;
    Rt.topLeftCorner(3,3) *= 0.09;
    Rt.bottomRightCorner(3,3) *= 0.4;
    x.setZero(6);
  }
  void reset(const Eigen::Vector3d& z) {
    x.head(3) = z;
    x.tail(3).setZero();
    Sigma.setZero();
    innovation_list.clear();
  }
  void predict() {
    x = A * x;
    Sigma = A * Sigma * A.transpose() + Qt;
  }
  void update(const Eigen::Vector3d& z_pos, const Eigen::Vector3d& z_vel) {
    Eigen::VectorXd z(6);
    z << z_pos, z_vel;
    Eigen::VectorXd y = z - C * x; // 新息
    innovation_list.push_back(y);
    if ((int)innovation_list.size() > window_size)
      innovation_list.pop_front();

    // 新息序列协方差估计
    Eigen::MatrixXd S_hat = Eigen::MatrixXd::Zero(6, 6);
    for (const auto& inn : innovation_list)
      S_hat += inn * inn.transpose();
    S_hat /= (double)innovation_list.size();

    // 自适应更新观测噪声协方差
    Eigen::MatrixXd S_pred = C * Sigma * C.transpose() + Rt;
    Eigen::MatrixXd Rt_adapt = S_hat - C * Sigma * C.transpose();
    // 保证正定性
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(Rt_adapt);
    Eigen::VectorXd eigvals = es.eigenvalues();
    for (int i = 0; i < eigvals.size(); ++i)
      if (eigvals(i) < 1e-6) eigvals(i) = 1e-6;
    Rt_adapt = es.eigenvectors() * eigvals.asDiagonal() * es.eigenvectors().transpose();

    Eigen::MatrixXd K = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt_adapt).inverse();
    x = x + K * y;
    Sigma = Sigma - K * C * Sigma;
  }
  Eigen::Vector3d pos() const { return x.head(3); }
  Eigen::Vector3d vel() const { return x.tail(3); }
};
