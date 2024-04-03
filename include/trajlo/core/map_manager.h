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

* Note:
* The `struct VoxelBlock`, `VoxelHash`, and functions `UpdateModelDeviation`,
* `ComputeModelError`, `ComputeThreshold`, which compute the threshold parameter
* for point registration, are copied from the work KISS-ICP
(https://github.com/PRBonn/kiss-icp),
* which is licensed under the MIT License.
*
* Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
Stachniss.
*
* The implementation of the map structure in this file is heavily inspired by
the work KISS-ICP.
* Thanks for their great effort and for open sourcing the code for the
community.

*/

#ifndef TRAJLO_MAP_MANAGER_H
#define TRAJLO_MAP_MANAGER_H

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tsl/robin_map.h>

#include <trajlo/utils/common_type.h>
#include <trajlo/utils/sophus_utils.hpp>

namespace traj {
class MapManager {
 public:
  using Ptr = std::shared_ptr<MapManager>;
  using Voxel = Eigen::Vector3i;

  // VoxelBlock and VoxelHash are copied from the work KISS-ICP
  // (https://github.com/PRBonn/kiss-icp),
  struct VoxelBlock {
    // buffer of points with a max limit of n_points
    std::vector<Eigen::Vector3d> points;
    int num_points_;

    inline void AddPoint(const Eigen::Vector3d &point) {
      if (points.size() < static_cast<size_t>(num_points_))
        points.push_back(point);
    }
  };

  struct VoxelHash {
    size_t operator()(const Voxel &voxel) const {
      const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
      return ((1 << 20) - 1) &
             (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
    }
  };

  MapManager(double ds_size, double voxel_size, double planer_threshold,
             int max_voxel_num, double max_range)
      : ds_size_(ds_size),
        voxel_size_(voxel_size),
        planer_threshold_(planer_threshold),
        max_points_per_voxel_(max_voxel_num),
        max_range_(max_range) {}

  std::vector<Eigen::Vector4d> DownSampling(
      const std::vector<Eigen::Vector4d> &points, double ds_size);

  void PreProcess(const std::vector<Eigen::Vector4d> &points,
                  const tStampPair &tp);

  void MapInit(const std::vector<Eigen::Vector4d> &points);

  void Update(const posePair &pp, const tStampPair &tp);

  struct ResultTuple {
    ResultTuple() {
      JTJ.setZero();
      JTr.setZero();
      error = 0;
      inlier = 0;
    }

    ResultTuple operator+(const ResultTuple &other) {
      this->JTJ += other.JTJ;
      this->JTr += other.JTr;
      this->error += other.error;
      this->inlier += other.inlier;
      return *this;
    }

    Eigen::Matrix<double, 12, 12> JTJ;
    Eigen::Matrix<double, 12, 1> JTr;
    double error;
    double inlier;
  };

  // find neighborhoods in seven voxels
  std::array<Eigen::Vector3i, 7> coord{
      Voxel(0, 0, 0),  Voxel(1, 0, 0),  Voxel(0, 1, 0), Voxel(0, 0, 1),
      Voxel(-1, 0, 0), Voxel(0, -1, 0), Voxel(0, 0, -1)};

  void PointRegistrationNormal(/*const posePair& pp,*/
                               const posePairLin &ppl, const tStampPair &tp,
                               Eigen::Matrix<double, 12, 12> &H_icp,
                               Eigen::Matrix<double, 12, 1> &b_icp,
                               double &error, double &inliers);

  inline bool IsInit() { return init_flag; }

  inline void SetInit() { init_flag = true; }

  /*
   * The threshold parameter for point registration part is
   * inspired by the work KISS-ICP https://github.com/PRBonn/kiss-icp
   * */
  inline void UpdateModelDeviation(const Sophus::SE3d &current_deviation) {
    model_deviation_ = current_deviation;
  }
  inline double ComputeModelError(const Sophus::SE3d &model_deviation,
                                  double max_range) {
    const double theta =
        Eigen::AngleAxisd(model_deviation.rotationMatrix()).angle();
    const double delta_rot = 2.0 * max_range * std::sin(theta / 2.0);
    const double delta_trans = model_deviation.translation().norm();
    return delta_trans + delta_rot;
  }
  inline void ComputeThreshold() {
    double model_error = ComputeModelError(model_deviation_, max_range_);
    if (model_error > min_motion_th_) {
      model_error_sse2_ += model_error * model_error;
      num_samples_++;
    }
    if (num_samples_ < 1) {
      reg_thresh_ = initial_threshold_;
    }
    reg_thresh_ = std::sqrt(model_error_sse2_ / num_samples_);
  }

 private:
  // adaptive threshhold
  double model_error_sse2_ = 0;
  int num_samples_ = 0;
  Sophus::SE3d model_deviation_ = Sophus::SE3d();
  double min_motion_th_ = 0.1;
  double initial_threshold_ = 1.5;
  double reg_thresh_ = 1.5;

  // map
  bool init_flag = false;
  double voxel_size_;
  double ds_size_;
  double max_range_ = 120;
  double planer_threshold_ = 0.01;
  int max_points_per_voxel_ = 20;
  tsl::robin_map<Voxel, VoxelBlock, VoxelHash> map;

  std::map<tStampPair, std::vector<Eigen::Vector4d>> reg_points_database;
  std::map<tStampPair, std::vector<Eigen::Vector4d>> map_points_database;
  int64_t last_scan_ts;
};

}  // namespace traj

#endif  // TRAJLO_MAP_MANAGER_H
