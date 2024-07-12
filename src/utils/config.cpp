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

#include <trajlo/utils/config.h>
#include <yaml-cpp/yaml.h>

namespace traj {
    void TrajConfig::load(const std::string &filename) {
        YAML::Node config = YAML::LoadFile(filename);

        // read dataset config
        type = config["dataset"]["data_type"].as<std::string>();
        topic=config["dataset"]["topic"].as<std::string>();
        dataset_path = config["dataset"]["path"].as<std::string>();

        save_pose=config["dataset"]["save_pose"].as<bool>();
        pose_file_path=config["dataset"]["pose_file_path"].as<std::string>();

        // read calibration config
        time_offset=config["calibration"]["time_offset"].as<double>();
        std::vector<double> tmp=config["calibration"]["T_body_lidar"].as<std::vector<double>>();
        Eigen::Matrix4d M44_lidar;
        M44_lidar<<tmp[0],tmp[1],tmp[2],tmp[3],
            tmp[4],tmp[5],tmp[6],tmp[7],
            tmp[8],tmp[9],tmp[10],tmp[11],
            tmp[12],tmp[13],tmp[14],tmp[15];

        T_body_lidar=Sophus::SE3d(M44_lidar);
        tmp=config["calibration"]["T_body_gt"].as<std::vector<double>>();
        Eigen::Matrix4d M44_gt;
        M44_gt<<tmp[0],tmp[1],tmp[2],tmp[3],
            tmp[4],tmp[5],tmp[6],tmp[7],
            tmp[8],tmp[9],tmp[10],tmp[11],
            tmp[12],tmp[13],tmp[14],tmp[15];
        T_body_gt=Sophus::SE3d(M44_gt);

        // read trajectory config
        init_interval = config["trajectory"]["init_interval"].as<double>();
        seg_interval = config["trajectory"]["seg_interval"].as<double>();
        seg_num = config["trajectory"]["seg_num"].as<int>();
        kinematic_constrain = config["trajectory"]["kinematic_constrain"].as<float>();
        init_pose_weight = config["trajectory"]["init_pose_weight"].as<double>();
        converge_thresh=config["trajectory"]["converge_thresh"].as<double>();
        max_iterations=config["trajectory"]["max_iterations"].as<int>();

        // read mapping config
        ds_size = config["mapping"]["ds_size"].as<float>();
        voxel_size = config["mapping"]["voxel_size"].as<float>();
        max_voxel_num = config["mapping"]["max_voxel_num"].as<int>();
        planer_thresh = config["mapping"]["planer_thresh"].as<float>();
        max_range = config["mapping"]["max_range"].as<float>();
        min_range = config["mapping"]["min_range"].as<float>();

        // vis
        frame_num=config["vis"]["frame_num"].as<int>();
        point_num=config["vis"]["point_num"].as<int>();

        tmp=config["vis"]["T_vis_lidar"].as<std::vector<double>>();
        Eigen::Matrix4d M44_vis;
        M44_vis<<tmp[0],tmp[1],tmp[2],tmp[3],
            tmp[4],tmp[5],tmp[6],tmp[7],
            tmp[8],tmp[9],tmp[10],tmp[11],
            tmp[12],tmp[13],tmp[14],tmp[15];
        T_vis_lidar=Sophus::SE3d(M44_vis);
    }

}