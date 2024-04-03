/*
 * This file is modified from the Basalt project.
 * https://gitlab.com/VladyslavUsenko/basalt-headers.git
 * which is under BSD 3-Clause License.
 * Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
 * */

#pragma once

#include "sophus/se3.hpp"

namespace Sophus {

template <typename Scalar>
SOPHUS_FUNC inline typename SE3<Scalar>::Tangent se3_logd(
    const SE3<Scalar> &se3) {
  typename SE3<Scalar>::Tangent upsilon_omega;
  upsilon_omega.template tail<3>() = se3.so3().log();
  upsilon_omega.template head<3>() = se3.translation();

  return upsilon_omega;
}

template <typename Derived>
SOPHUS_FUNC inline SE3<typename Derived::Scalar> se3_expd(
    const Eigen::MatrixBase<Derived> &upsilon_omega) {
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 6);

  using Scalar = typename Derived::Scalar;

  return SE3<Scalar>(SO3<Scalar>::exp(upsilon_omega.template tail<3>()),
                     upsilon_omega.template head<3>());
}

template <typename Derived1, typename Derived2>
SOPHUS_FUNC inline void rightJacobianSO3(
    const Eigen::MatrixBase<Derived1> &phi,
    const Eigen::MatrixBase<Derived2> &J_phi) {
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 3);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 3, 3);

  using Scalar = typename Derived1::Scalar;

  using std::cos;
  using std::sin;
  using std::sqrt;

  Eigen::MatrixBase<Derived2> &J =
      const_cast<Eigen::MatrixBase<Derived2> &>(J_phi);

  Scalar phi_norm2 = phi.squaredNorm();
  Eigen::Matrix<Scalar, 3, 3> phi_hat = Sophus::SO3<Scalar>::hat(phi);
  Eigen::Matrix<Scalar, 3, 3> phi_hat2 = phi_hat * phi_hat;

  J.setIdentity();

  if (phi_norm2 > Sophus::Constants<Scalar>::epsilon()) {
    Scalar phi_norm = sqrt(phi_norm2);
    Scalar phi_norm3 = phi_norm2 * phi_norm;

    J -= phi_hat * (1 - cos(phi_norm)) / phi_norm2;
    J += phi_hat2 * (phi_norm - sin(phi_norm)) / phi_norm3;
  } else {
    // Taylor expansion around 0
    J -= phi_hat / 2;
    J += phi_hat2 / 6;
  }
}

template <typename Derived1, typename Derived2>
SOPHUS_FUNC inline void rightJacobianInvSO3(
    const Eigen::MatrixBase<Derived1> &phi,
    const Eigen::MatrixBase<Derived2> &J_phi) {
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 3);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 3, 3);

  using Scalar = typename Derived1::Scalar;

  using std::cos;
  using std::sin;
  using std::sqrt;

  Eigen::MatrixBase<Derived2> &J =
      const_cast<Eigen::MatrixBase<Derived2> &>(J_phi);

  Scalar phi_norm2 = phi.squaredNorm();
  Eigen::Matrix<Scalar, 3, 3> phi_hat = Sophus::SO3<Scalar>::hat(phi);
  Eigen::Matrix<Scalar, 3, 3> phi_hat2 = phi_hat * phi_hat;

  J.setIdentity();
  J += phi_hat / 2;

  if (phi_norm2 > Sophus::Constants<Scalar>::epsilon()) {
    Scalar phi_norm = sqrt(phi_norm2);

    // We require that the angle is in range [0, pi]. We check if we are close
    // to pi and apply a Taylor expansion to scalar multiplier of phi_hat2.
    // Technically, log(exp(phi)exp(epsilon)) is not continuous / differentiable
    // at phi=pi, but we still aim to return a reasonable value for all valid
    // inputs.
    //    BASALT_ASSERT(phi_norm <= M_PI +
    //    Sophus::Constants<Scalar>::epsilon());

    if (phi_norm < M_PI - Sophus::Constants<Scalar>::epsilonSqrt()) {
      // regular case for range (0,pi)
      J += phi_hat2 * (1 / phi_norm2 -
                       (1 + cos(phi_norm)) / (2 * phi_norm * sin(phi_norm)));
    } else {
      // 0th-order Taylor expansion around pi
      J += phi_hat2 / (M_PI * M_PI);
    }
  } else {
    // Taylor expansion around 0
    J += phi_hat2 / 12;
  }
}

template <typename Derived1, typename Derived2>
SOPHUS_FUNC inline void leftJacobianSO3(
    const Eigen::MatrixBase<Derived1> &phi,
    const Eigen::MatrixBase<Derived2> &J_phi) {
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 3);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 3, 3);

  using Scalar = typename Derived1::Scalar;

  using std::cos;
  using std::sin;
  using std::sqrt;

  Eigen::MatrixBase<Derived2> &J =
      const_cast<Eigen::MatrixBase<Derived2> &>(J_phi);

  Scalar phi_norm2 = phi.squaredNorm();
  Eigen::Matrix<Scalar, 3, 3> phi_hat = Sophus::SO3<Scalar>::hat(phi);
  Eigen::Matrix<Scalar, 3, 3> phi_hat2 = phi_hat * phi_hat;

  J.setIdentity();

  if (phi_norm2 > Sophus::Constants<Scalar>::epsilon()) {
    Scalar phi_norm = sqrt(phi_norm2);
    Scalar phi_norm3 = phi_norm2 * phi_norm;

    J += phi_hat * (1 - cos(phi_norm)) / phi_norm2;
    J += phi_hat2 * (phi_norm - sin(phi_norm)) / phi_norm3;
  } else {
    // Taylor expansion around 0
    J += phi_hat / 2;
    J += phi_hat2 / 6;
  }
}

template <typename Derived1, typename Derived2>
SOPHUS_FUNC inline void leftJacobianInvSO3(
    const Eigen::MatrixBase<Derived1> &phi,
    const Eigen::MatrixBase<Derived2> &J_phi) {
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 3);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 3, 3);

  using Scalar = typename Derived1::Scalar;

  using std::cos;
  using std::sin;
  using std::sqrt;

  Eigen::MatrixBase<Derived2> &J =
      const_cast<Eigen::MatrixBase<Derived2> &>(J_phi);

  Scalar phi_norm2 = phi.squaredNorm();
  Eigen::Matrix<Scalar, 3, 3> phi_hat = Sophus::SO3<Scalar>::hat(phi);
  Eigen::Matrix<Scalar, 3, 3> phi_hat2 = phi_hat * phi_hat;

  J.setIdentity();
  J -= phi_hat / 2;

  if (phi_norm2 > Sophus::Constants<Scalar>::epsilon()) {
    Scalar phi_norm = sqrt(phi_norm2);

    // We require that the angle is in range [0, pi]. We check if we are close
    // to pi and apply a Taylor expansion to scalar multiplier of phi_hat2.
    // Technically, log(exp(phi)exp(epsilon)) is not continuous / differentiable
    // at phi=pi, but we still aim to return a reasonable value for all valid
    // inputs.
    //    BASALT_ASSERT(phi_norm <= M_PI +
    //    Sophus::Constants<Scalar>::epsilon());

    if (phi_norm < M_PI - Sophus::Constants<Scalar>::epsilonSqrt()) {
      // regular case for range (0,pi)
      J += phi_hat2 * (1 / phi_norm2 -
                       (1 + cos(phi_norm)) / (2 * phi_norm * sin(phi_norm)));
    } else {
      // 0th-order Taylor expansion around pi
      J += phi_hat2 / (M_PI * M_PI);
    }
  } else {
    // Taylor expansion around 0
    J += phi_hat2 / 12;
  }
}

template <typename Derived1, typename Derived2>
SOPHUS_FUNC inline void rightJacobianSE3Decoupled(
    const Eigen::MatrixBase<Derived1> &phi,
    const Eigen::MatrixBase<Derived2> &J_phi) {
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 6);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 6, 6);

  using Scalar = typename Derived1::Scalar;

  Eigen::MatrixBase<Derived2> &J =
      const_cast<Eigen::MatrixBase<Derived2> &>(J_phi);

  J.setZero();

  Eigen::Matrix<Scalar, 3, 1> omega = phi.template tail<3>();
  rightJacobianSO3(omega, J.template bottomRightCorner<3, 3>());
  J.template topLeftCorner<3, 3>() =
      Sophus::SO3<Scalar>::exp(omega).inverse().matrix();
}

template <typename Derived1, typename Derived2>
SOPHUS_FUNC inline void rightJacobianInvSE3Decoupled(
    const Eigen::MatrixBase<Derived1> &phi,
    const Eigen::MatrixBase<Derived2> &J_phi) {
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived1);
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived2);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived1, 6);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 6, 6);

  using Scalar = typename Derived1::Scalar;

  Eigen::MatrixBase<Derived2> &J =
      const_cast<Eigen::MatrixBase<Derived2> &>(J_phi);

  J.setZero();

  Eigen::Matrix<Scalar, 3, 1> omega = phi.template tail<3>();
  rightJacobianInvSO3(omega, J.template bottomRightCorner<3, 3>());
  J.template topLeftCorner<3, 3>() = Sophus::SO3<Scalar>::exp(omega).matrix();
}
}  // namespace Sophus
