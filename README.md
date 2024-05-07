<div align="center">
    <h1>Traj-LO</h1>
    <i>A LiDAR-only Odometry from Continuous-Time perspective</i>
    <br>
    <br>
<div>

[🛠️ Installation](#how-to-use-traj-lo) |
[🎥 Video](https://youtu.be/hbtKzElYKkQ?si=ZlqvtUVhhJbAju0S) |
[📖 Paper](https://ieeexplore.ieee.org/document/10387726)

</div>
    <br>
    <img src="doc/image/trajectory.png" width="50%" height="auto" alt="Trajectory Image">
    <img src="doc/image/pipeline.png" width="40%" height="auto" alt="Pipeline Image">
<br>
</div>




## What is Traj-LO
**Traj-LO aims to explore the limits of state estimation using only LiDAR sensors.**

Nowadays, LO approaches heavily rely on IMU data for accurate state estimation but neglect the true capabilities of LiDAR sensors. Through the continuous-time perspective, Traj-LO matches the performance of state-of-the-art LIO methods in diverse scenarios.

The spatial-temporal movement of LiDAR is parameterized by a simple yet effective continuous-time trajectory, which consists of multiple piecewise linear functions.
By coupling the geometric information from streaming LiDAR points and kinematic constraints from trajectory smoothness, it can work even in scenarios where the motion state exceeds the IMU's measuring range.
Besides, the framework is generalized for different kinds of LiDAR as well as multi-LiDAR systems.
## How to use Traj-LO
Traj-LO is a ROS-independent project and is suitable for cross-platform applications. For convenience, we provide a ROSbag data loader that can read public datasets and your own recorded data.

### Supported Dataset
Currently, the released code only supports one LiDAR configuration. We will update it as soon as possible to provide multi-LiDAR support. The provided ROSbag data loader supports different types of LiDAR, including Livox, Ouster, Hesai, Robosense, and Velodyne. We have tested Traj-LO with the following datasets.

| Dataset                                                                                                                                                                                                                 | LiDAR                               | Message Type                                       | Configuration file                       |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------|----------------------------------------------------|------------------------------------------|
| [NTU VIRAL](https://ntu-aris.github.io/ntu_viral_dataset/)                                                                                                                                                              | Two Ouster OS1-16                   | sensor_msgs/PointCloud2                            | config_ntu.yaml                          |
| [Hilti 2021](https://www.hilti-challenge.com/dataset-2021.html)                                                                                                                                                         | Ouster OS0-64 <br/> Livox mid-70    | sensor_msgs/PointCloud2 <br/>livox_ros_driver/CustomMsg | config_ouster.yaml<br/>config_livox.yaml |
| [R3LIVE](https://github.com/ziv-lin/r3live_dataset)                                                                                                                                                                     | Livox Avia                          | livox_ros_driver/CustomMsg                            | config_livox.yaml                        |
| [Point-LIO](https://connecthkuhk-my.sharepoint.com/personal/hdj65822_connect_hku_hk/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Fhdj65822%5Fconnect%5Fhku%5Fhk%2FDocuments%2FDataset%20for%20Point%2DLIO%20examples&ga=1) | Livox Avia                          | livox_ros_driver/CustomMsg                            | config_pointlio.yaml                     |
| [New College](https://ori-drs.github.io/newer-college-dataset/)                                                                                                                                                         | Ouster OS-1 64 <br/>Ouster OS-0 128 | sensor_msgs/PointCloud2                            | config_ouster.yaml                       |
| [Hilti 2022 &2023](https://www.hilti-challenge.com/dataset-2022.html)                                                                                                                                                    | Hesai PandarXT-32 | sensor_msgs/PointCloud2                            | config_hesai.yaml                        |
| [SubT-MRS](https://superodometry.com/iccv23_challenge_LiI)                                                                                                                                                    | Velodyne VLP16 | sensor_msgs/PointCloud2                            | config_velodyne.yaml                     |

The corresponding configuration files are located in the "data" directory. For optimal performance, you will need to fine-tune the parameters.

Since Traj-LO is a LiDAR-only method, it may fail in narrow spaces where there are few valid points for a long time.

### Dependency
In addition to the ROSbag data loader, Traj-LO also provides a simple custom GUI for visualization and uses Eigen-based Gauss-Newton for pose optimization. Here are the major libraries we will use.
- Optimization: [Eigen](https://gitlab.com/libeigen/eigen.git), [Sophus](https://github.com/strasdat/Sophus.git)
- GUI: [ImGui](https://github.com/ocornut/imgui), OpenGL, [GLM](https://github.com/g-truc/glm.git)
- DataLoader: oneTBB, Boost

Although major dependencies are included in the third-party folder, you may still need to run the script `install_deps.sh` to install libraries like Boost, etc.
### Build
You can install the Traj-LO project by following these steps:
```
git clone --recursive https://github.com/kevin2431/Traj-LO.git
cd Traj-LO
sh scripts/install_deps.sh # make sure we have all the dependency
mkdir build && cd build
cmake .. 
make -j8
```



### Run
After modifying the config file for your environment, you can run Traj-LO. Here is an example to test it with a Livox LiDAR.
```
./trajlo ../data/config_livox.yaml
```

### Some Tips
- Traj-LO is a continuous-time method, so each point in your rosbag should have a corresponding timestamp.
- When the motion profile is aggressive, you can decrease `seg_interval` or increase `kinematic_constraint`.

## Cross-platform Support
### Linux
Ubuntu 20.04, 22.04
### Windows
You can use [WSL2](https://learn.microsoft.com/zh-cn/windows/wsl/about) to install the Ubuntu subsystem and then follow the above instructions to test Traj-LO. To enable OpenGL accelerated rendering in WSLg, you may need to [select Nvidia GPU](https://github.com/microsoft/wslg/wiki/GPU-selection-in-WSLg).
### MacOS
Make sure you have [Homebrew](https://brew.sh/) to run the srcipt `install_deps.sh`to install dependencies. We have tested Traj-LO on M2 Mac Mini (macOS 14.4.1).
### ROS
Still working on it!

## Citation

If you use this project for any academic work, please cite our RA-L [paper](https://ieeexplore.ieee.org/document/10387726).

```bibtex
@ARTICLE{zheng2024traj,
    author={Zheng, Xin and Zhu, Jianke},
    journal={IEEE Robotics and Automation Letters},
    title={Traj-LO: In Defense of LiDAR-Only
    Odometry Using an Effective Continuous-Time
    Trajectory},
    year={2024},
    volume={9},
    number={2},
    pages={1961-1968},
    doi={10.1109/LRA.2024.3352360}
}
```

For those interested in multi-sensor fusion, particularly in multi-LiDAR and multi-IMU systems, I recommend checking out my latest work, [Traj-LIO](https://arxiv.org/abs/2402.09189), which is a resilient state estimator through sparse Gaussian Processes.

## Contributing

Traj-LO is currently in beta version, and we are actively working on it. We welcome community users to participate in this project.

## Acknowledgement
Thanks for these pioneering works [Basalt](https://cvg.cit.tum.de/research/vslam/basalt) (Batch Optimization), [CT-ICP](https://github.com/jedeschaud/ct_icp) (Continuous-time Idea), and [KISS-ICP](https://github.com/PRBonn/kiss-icp) (VoxelMap Management).

[![Star History Chart](https://api.star-history.com/svg?repos=kevin2431/Traj-LO&type=Date)](https://star-history.com/#kevin2431/Traj-LO&Date)







