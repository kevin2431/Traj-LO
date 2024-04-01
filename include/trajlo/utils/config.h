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

#ifndef TRAJLO_CONFIG_H
#define TRAJLO_CONFIG_H

#include <string>
#include <vector>

namespace traj {
struct TrajConfig {
  TrajConfig() = default;
  void load(const std::string& filename);

  // dataset
  std::string type;
  std::string topic;
  std::string dataset_path;

  // trajectory
  double init_interval;
  double seg_interval;
  int seg_num;
  float kinematic_constrain;
  double init_pose_weight;
  double converge_thresh;
  int max_iterations;

  // mapping
  float ds_size;
  float voxel_size;
  int max_voxel_num;
  float planer_thresh;
  float max_range;
  float min_range;

  // vis
  int frame_num;
  int point_num;
//  int width;
//  int height;
};
}  // namespace traj

#endif  // TRAJLO_CONFIG_H
