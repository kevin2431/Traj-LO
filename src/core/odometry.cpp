/**
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

#include <trajlo/core/odometry.h>
#include <trajlo/utils/sophus_utils.hpp>

#include <fstream>
#include <iomanip>

uint64_t getCurrTime() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::high_resolution_clock::now().time_since_epoch())
      .count();
}

namespace traj {
TrajLOdometry::TrajLOdometry(const TrajConfig& config)
    : config_(config), isFinish(false) {
  min_range_2_ = config.min_range * config.min_range;
  converge_thresh_ = config.converge_thresh;

  laser_data_queue.set_capacity(100);

  init_interval_ = config.init_interval;
  window_interval_ = config.seg_interval;
  max_frames_ = config.seg_num;

  map_.reset(new MapManager(config.ds_size, config.voxel_size,
                            config.planer_thresh, config.max_voxel_num,
                            config.max_range));

  // setup marginalization
  marg_H.setZero(POSE_SIZE, POSE_SIZE);
  marg_b.setZero(POSE_SIZE);
  double init_pose_weight = config.init_pose_weight;
  marg_H.diagonal().setConstant(init_pose_weight);
}

void TrajLOdometry::Start() {
  auto lo_func = [&] {
    int frame_id = 0;
    Scan::Ptr curr_scan;
    bool first_scan = true;
    Measurement::Ptr measure;

    while (true) {
      /*
       * this thread will block until the valid scan coming
       * */
      laser_data_queue.pop(curr_scan);
      if (!curr_scan.get()) break;

      if (first_scan) {
        last_begin_t_ns_ = curr_scan->timestamp;
        last_end_t_ns_ = last_begin_t_ns_ + init_interval_;
        first_scan = false;
      }
      PointCloudSegment(curr_scan, measure);

      while (!measure_cache.empty()) {
        measure = measure_cache.front();
        measure_cache.pop_front();

        // 1. range filter & compute the relative timestamps
        std::vector<Eigen::Vector4d> points;
        RangeFilter(measure, points);

        const auto& tp = measure->tp;
        if (!map_->IsInit()) {
          T_wc_curr = Sophus::SE3d();
          map_->MapInit(points);

          // standing start
          frame_poses_[tp.second] =
              PoseStateWithLin<double>(tp.second, T_wc_curr, true);
          trajectory_.emplace_back(tp.first, T_wc_curr);
          map_->SetInit();

          T_prior = Sophus::SE3d();
        } else {
          measure->pseudoPrior = T_prior;
          measurements[tp] = measure;
          Sophus::SE3d T_w_pred = frame_poses_[tp.first].getPose() * T_prior;
          frame_poses_[tp.second] =
              PoseStateWithLin<double>(tp.second, T_w_pred);

          // 2. preprocess the point cloud.
          map_->PreProcess(points, tp);

          if (!isMove_) {
            isMove_ = (T_w_pred).translation().norm() > 0.5;
          } else {
            map_->ComputeThreshold();
          }

          // 3. find the optimal control poses based on the geometric and motion
          // constrains
          Optimize();

          T_wc_curr = frame_poses_[tp.second].getPose();
          Sophus::SE3d model_deviation = T_w_pred.inverse() * T_wc_curr;
          map_->UpdateModelDeviation(model_deviation);
          T_prior = frame_poses_[tp.first].getPose().inverse() *
                    frame_poses_[tp.second].getPose();

          // 4. marginalize the oldest segment and update the map using points
          // belond to the oldest segment.
          Marginalize();

          // map & trajectory visualization
          if (vis_data_queue &&
              ((frame_id / config_.seg_num) % (config_.frame_num) == 0)) {
            ScanVisData::Ptr visData(new ScanVisData);

            posePair pp{frame_poses_[tp.first].getPose(),
                        frame_poses_[tp.second].getPose()};
            UndistortRawPoints(measure->points, visData->data, pp);

            visData->T_w = pp.first;
            vis_data_queue->push(visData);  // may block the thread
          }
        }
        frame_id++;
      }
    }

    if (vis_data_queue) vis_data_queue->push(nullptr);

    // save pose in window
    for (const auto& kv : frame_poses_) {
      trajectory_.emplace_back(kv.first, kv.second.getPose());
    }

    // Here, you can save the trajectory for comparison
    if(config_.save_pose){
      std::cout << "Start Pose Saving!" << std::endl;
      std::ofstream os(config_.pose_file_path);
      os << "# timestamp tx ty tz qx qy qz qw" << std::endl;

      for(const auto& p:trajectory_){
        Sophus::SE3d pose_body=config_.T_body_lidar*p.second*config_.T_body_lidar.inverse();
        Sophus::SE3d pose_gt=pose_body*config_.T_body_gt;

        os << std::scientific << std::setprecision(18)
           << p.first * 1e-9 << " " << pose_gt.translation().x()
           << " " << pose_gt.translation().y() << " "
           << pose_gt.translation().z() << " "
           << pose_gt.so3().unit_quaternion().x() << " "
           << pose_gt.so3().unit_quaternion().y() << " "
           << pose_gt.so3().unit_quaternion().z() << " "
           << pose_gt.so3().unit_quaternion().w() << std::endl;
      }
      os.close();
      std::cout << "Finish Pose Saving!" << std::endl;
    }

    isFinish = true;
    std::cout << "Finisher LiDAR Odometry " << std::endl;
  };

  processing_thread_.reset(new std::thread(lo_func));
}

/*
 * The analytic Jacobians in the paper are derived in SE(3) form. For the
 * efficiency in our implementation, we instead update poses in SO(3)+R3
 * form. The connection between them has been discussed in
 * https://gitlab.com/VladyslavUsenko/basalt/-/issues/37
 * */
void TrajLOdometry::Optimize() {
  AbsOrderMap aom;
  for (const auto& kv : frame_poses_) {
    aom.abs_order_map[kv.first] = std::make_pair(aom.total_size, POSE_SIZE);
    aom.total_size += POSE_SIZE;
    aom.items++;
  }

  Eigen::MatrixXd abs_H;
  Eigen::VectorXd abs_b;

  for (int iter = 0; iter < config_.max_iterations; iter++) {
    abs_H.setZero(aom.total_size, aom.total_size);
    abs_b.setZero(aom.total_size);

    // 两帧优化
    for (auto& m : measurements) {
      int64_t idx_prev = m.first.first;
      int64_t idx_curr = m.first.second;

      const auto& prev = frame_poses_[idx_prev];
      const auto& curr = frame_poses_[idx_curr];

      posePair pp{prev.getPose(), curr.getPose()};
      const tStampPair& tp = m.second->tp;  //{idx_prev,idx_curr};

      // 1. Geometric constrains from lidar point cloud.
      map_->PointRegistrationNormal({prev, curr}, tp, m.second->delta_H,
                                    m.second->delta_b, m.second->lastError,
                                    m.second->lastInliers);

      // 2. Motion constrains behind continuous movement.
      // Log(Tbe)-Log(prior) Equ.(6)
      {
        Sophus::SE3d T_be = pp.first.inverse() * pp.second;
        Sophus::Vector6d tau = Sophus::se3_logd(T_be);
        Sophus::Vector6d res = tau - Sophus::se3_logd(m.second->pseudoPrior);

        Sophus::Matrix6d J_T_w_b;
        Sophus::Matrix6d J_T_w_e;
        Sophus::Matrix6d rr_b;
        Sophus::Matrix6d rr_e;

        if (prev.isLinearized() || curr.isLinearized()) {
          pp = std::make_pair(prev.getPoseLin(), curr.getPoseLin());
          T_be = pp.first.inverse() * pp.second;
          tau = Sophus::se3_logd(T_be);
        }

        Sophus::rightJacobianInvSE3Decoupled(tau, J_T_w_e);
        J_T_w_b = -J_T_w_e * (T_be.inverse()).Adj();

        rr_b.setIdentity();
        rr_b.topLeftCorner<3, 3>() = pp.first.rotationMatrix().transpose();
        rr_e.setIdentity();
        rr_e.topLeftCorner<3, 3>() = pp.second.rotationMatrix().transpose();

        Eigen::Matrix<double, 6, 12> J_be;
        J_be.topLeftCorner<6, 6>() = J_T_w_b * rr_b;
        J_be.topRightCorner<6, 6>() = J_T_w_e * rr_e;

        double alpha_e = config_.kinematic_constrain * m.second->lastInliers;
        m.second->delta_H += alpha_e * J_be.transpose() * J_be;
        m.second->delta_b -= alpha_e * J_be.transpose() * res;
      }

      int abs_id = aom.abs_order_map.at(idx_prev).first;
      abs_H.block<POSE_SIZE * 2, POSE_SIZE * 2>(abs_id, abs_id) +=
          m.second->delta_H;
      abs_b.segment<POSE_SIZE * 2>(abs_id) += m.second->delta_b;
    }

    // Marginalization Error Term
    // reference: Square Root Marginalization for Sliding-Window Bundle
    // Adjustment (N Demmel, D Schubert, C Sommer, D Cremers and V Usenko)
    // https://arxiv.org/abs/2109.02182
    Eigen::VectorXd delta;
    for (const auto& p : frame_poses_) {
      if (p.second.isLinearized()) {
        delta = p.second.getDelta();
      }
    }
    abs_H.block<POSE_SIZE, POSE_SIZE>(0, 0) += marg_H;
    abs_b.head<POSE_SIZE>() -= marg_b;
    abs_b.head<POSE_SIZE>() -= (marg_H * delta);

    Eigen::VectorXd update = abs_H.ldlt().solve(abs_b);
    double max_inc = update.array().abs().maxCoeff();

    if (max_inc < converge_thresh_) {
      break;
    }

    for (auto& kv : frame_poses_) {
      int idx = aom.abs_order_map.at(kv.first).first;
      kv.second.applyInc(update.segment<POSE_SIZE>(idx));
    }
  }

  // update pseudo motion prior after each optimization
  int64_t begin_t = measurements.begin()->first.first;
  int64_t end_t = measurements.begin()->first.second;
  auto begin = frame_poses_[begin_t];
  auto end = frame_poses_[end_t];

  const int64_t m0_t = begin_t;
  for (auto m : measurements) {
    if (m.first.first == m0_t) continue;
    m.second->pseudoPrior = begin.getPose().inverse() * end.getPose();
    begin = frame_poses_[m.first.first];
    end = frame_poses_[m.first.second];
  }
}

void TrajLOdometry::Marginalize() {
  // remove pose with minimal timestamp
  if (measurements.size() >= max_frames_) {
    const auto& tp = measurements.begin()->first;
    const posePair pp{frame_poses_[tp.first].getPose(),
                      frame_poses_[tp.second].getPose()};
    map_->Update(pp, tp);

    Eigen::VectorXd delta = frame_poses_[tp.first].getDelta();

    Eigen::Matrix<double, 12, 12> marg_H_new =
        measurements.begin()->second->delta_H;
    Eigen::Matrix<double, 12, 1> marg_b_new =
        measurements.begin()->second->delta_b;
    marg_H_new.topLeftCorner<POSE_SIZE, POSE_SIZE>() += marg_H;

    marg_b_new.head<POSE_SIZE>() -= marg_b;
    marg_b_new.head<POSE_SIZE>() -= (marg_H * delta);

    Eigen::MatrixXd H_mm_inv =
        marg_H_new.topLeftCorner<6, 6>().fullPivLu().solve(
            Eigen::MatrixXd::Identity(6, 6));
    marg_H_new.bottomLeftCorner<6, 6>() *= H_mm_inv;

    marg_H = marg_H_new.bottomRightCorner<6, 6>();
    marg_b = marg_b_new.tail<6>();
    marg_H -=
        marg_H_new.bottomLeftCorner<6, 6>() * marg_H_new.topRightCorner<6, 6>();
    marg_b -= marg_H_new.bottomLeftCorner<6, 6>() * marg_b_new.head<6>();


    // erase
    frame_poses_.erase(tp.first);
    measurements.erase(tp);

    trajectory_.emplace_back(tp.first, pp.first);
    frame_poses_[tp.second].setLinTrue();
  }
}

void TrajLOdometry::PointCloudSegment(Scan::Ptr scan,
                                      Measurement::Ptr measure) {
  for (size_t i = 0; i < scan->size; i++) {
    const auto& p = scan->points[i];
    if (static_cast<int64_t>(p.ts * 1e9) < last_end_t_ns_) {
      points_cache.emplace_back(p);
    } else {
      // pub one measurement
      measure.reset(new Measurement);
      measure->tp = {last_begin_t_ns_, last_end_t_ns_};
      measure->points = points_cache;
      measure_cache.push_back(measure);

      // reset cache and time
      points_cache.clear();
      last_begin_t_ns_ = last_end_t_ns_;
      last_end_t_ns_ = last_begin_t_ns_ + window_interval_;

      if (static_cast<int64_t>(p.ts * 1e9) < last_end_t_ns_) {
        points_cache.emplace_back(p);
      }
    }
  }
}

void TrajLOdometry::RangeFilter(Measurement::Ptr measure,
                                std::vector<Eigen::Vector4d>& points) {
  points.reserve(measure->points.size());
  const auto& tp = measure->tp;
  double interv = (tp.second - tp.first) * 1e-9;
  double begin_ts = tp.first * 1e-9;
  for (const auto& p : measure->points) {
    if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) continue;
    double len = (p.x * p.x + p.y * p.y + p.z * p.z);
    if (len < min_range_2_) continue;

    double alpha = (p.ts - begin_ts) / interv;
    points.emplace_back(Eigen::Vector4d(p.x, p.y, p.z, alpha));
  }
}

void TrajLOdometry::UndistortRawPoints(std::vector<PointXYZIT>& pc_in,
                                       std::vector<PointXYZI>& pc_out,
                                       const posePair& pp) {
  Sophus::Vector6f tau =
      Sophus::se3_logd(pp.first.inverse() * pp.second).cast<float>();

  double interv = (pc_in.back().ts - pc_in.front().ts);
  double begin_ts = pc_in.front().ts;

  pc_out.reserve(pc_in.size());
  int i = 0;
  for (const auto& p : pc_in) {
    if (i % config_.point_num == 0) {
      //        float alpha = i * 1.0f / num;
      float alpha = (p.ts - begin_ts) / interv;
      Eigen::Vector3f point(p.x, p.y, p.z);
      if (point.hasNaN() || point.squaredNorm() < 4) continue;

      Sophus::SE3f T_b_i = Sophus::se3_expd(alpha * tau);
      point = T_b_i * point;
      PointXYZI po{point(0), point(1), point(2), p.intensity};
      pc_out.emplace_back(po);
    }
    i++;
  }
}

}  // namespace traj
