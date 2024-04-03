/**
This file is modified from the Basalt project,
https://gitlab.com/VladyslavUsenko/basalt-headers.git,
which is under BSD 3-Clause License.
Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.

MIT License

Copyright (c) 2023 Xin Zheng <xinzheng@zju.edu.cn>.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef TRAJLO_POSE_TYPE_H
#define TRAJLO_POSE_TYPE_H

#include <sophus/se3.hpp>
#include <iostream>
#include <map>

namespace traj{
    constexpr size_t POSE_SIZE = 6;  ///< Dimentionality of the pose state
    constexpr size_t POSE_VEL_SIZE =
            9;  ///< Dimentionality of the pose-velocity state
    constexpr size_t POSE_VEL_BIAS_SIZE =
            15;  ///< Dimentionality of the pose-velocity-bias state

/// @brief State that consists of SE(3) pose at a certain time.
    template <class Scalar_>
    struct PoseState {
        using Scalar = Scalar_;
        using VecN = Eigen::Matrix<Scalar, POSE_SIZE, 1>;
        using Vec6 = Eigen::Matrix<Scalar, 6, 1>;
        using SO3 = Sophus::SO3<Scalar>;
        using SE3 = Sophus::SE3<Scalar>;

        /// @brief Default constructor with Identity pose and zero timestamp.
        PoseState() { t_ns = 0; }

        /// @brief Constructor with timestamp and pose.
        ///
        /// @param t_ns timestamp of the state in nanoseconds
        /// @param T_w_i transformation from the body frame to the world frame
        PoseState(int64_t t_ns, const SE3& T_w_i) : t_ns(t_ns), T_w_i(T_w_i) {}

        /// @brief Create copy with different Scalar type.
        template <class Scalar2>
        PoseState<Scalar2> cast() const {
            PoseState<Scalar2> a;
            a.t_ns = t_ns;
            a.T_w_i = T_w_i.template cast<Scalar2>();
            return a;
        }

        /// @brief Apply increment to the pose
        ///
        /// For details see \ref incPose
        /// @param[in] inc 6x1 increment vector
        void applyInc(const VecN& inc) { incPose(inc, T_w_i); }

        /// @brief Apply increment to the pose
        ///
        /// The incremernt vector consists of translational and rotational parts \f$
        /// [\upsilon, \omega]^T \f$. Given the current pose \f$ R \in
        /// SO(3), p \in \mathbb{R}^3\f$ the updated pose is: \f{align}{ R' &=
        /// \exp(\omega) R
        /// \\ p' &= p + \upsilon
        /// \f}
        ///  The increment is consistent with \ref
        /// Se3Spline::applyInc.
        ///
        /// @param[in] inc 6x1 increment vector
        /// @param[in,out] T the pose to update
        inline static void incPose(const Vec6& inc, SE3& T) {
            // use right update, consistent with gtsam
            T.translation() += inc.template head<3>();
            T.so3()=T.so3()* SO3::exp(inc.template tail<3>());
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                int64_t t_ns;  ///< timestamp of the state in nanoseconds
        SE3 T_w_i;     ///< pose of the state
    };

    template <class Scalar_>
    struct PoseStateWithLin {
        using Scalar = Scalar_;
        using VecN = typename PoseState<Scalar>::VecN;
        using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
        using SE3 = Sophus::SE3<Scalar>;

        PoseStateWithLin() {
            linearized = false;
            delta.setZero();
        };

        PoseStateWithLin(int64_t t_ns, const SE3& T_w_i, bool linearized = false)
                : linearized(linearized), pose_linearized(t_ns, T_w_i) {
            delta.setZero();
            T_w_i_current = T_w_i;
        }


        template <class Scalar2>
        PoseStateWithLin<Scalar2> cast() const {
            PoseStateWithLin<Scalar2> a;
            a.linearized = linearized;
            a.delta = delta.template cast<Scalar2>();
            a.pose_linearized = pose_linearized.template cast<Scalar2>();
            a.T_w_i_current = T_w_i_current.template cast<Scalar2>();
            return a;
        }

        void setLinTrue() {
            linearized = true;
            T_w_i_current = pose_linearized.T_w_i;
        }

        inline const SE3& getPose() const {
            if (!linearized) {
                return pose_linearized.T_w_i;
            } else {
                return T_w_i_current;
            }
        }

        inline const SE3& getPoseLin() const { return pose_linearized.T_w_i; }

        inline void applyInc(const VecN& inc) {
            if (!linearized) {
                PoseState<Scalar>::incPose(inc, pose_linearized.T_w_i);
            } else {
                delta += inc;
                T_w_i_current = pose_linearized.T_w_i;
                PoseState<Scalar>::incPose(delta, T_w_i_current);
            }
        }

        inline bool isLinearized() const { return linearized; }
        inline const VecN& getDelta() const { return delta; }
        inline int64_t getT_ns() const { return pose_linearized.t_ns; }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        bool linearized;
        VecN delta;
        PoseState<Scalar> pose_linearized;
        SE3 T_w_i_current;

        VecN backup_delta;
        PoseState<Scalar> backup_pose_linearized;
        SE3 backup_T_w_i_current;

        // give access to private members for cast() implementation
        template <class>
        friend struct PoseStateWithLin;
    };


    struct AbsOrderMap {
        std::map<int64_t, std::pair<int, int>> abs_order_map;
        size_t items = 0;
        size_t total_size = 0;

        void print_order() {
            for (const auto& kv : abs_order_map) {
                std::cout << kv.first << " (" << kv.second.first << ","
                          << kv.second.second << ")" << std::endl;
            }
            std::cout << std::endl;
        }
    };
}


#endif //TRAJLO_POSE_TYPE_H
