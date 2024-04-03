<div align="center">
    <h1>Traj-LO</h1>
    <i>A LiDAR-only Odometry from Continuous-Time perspective</i>
    <br>
    <br>
    <a href=https://youtu.be/hbtKzElYKkQ?si=ZlqvtUVhhJbAju0S>Video</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://ieeexplore.ieee.org/document/10387726">Paper</a>
    <br>
    <br>
    <img src="doc/image/trajectory.png" width="50%" height="auto" alt="Trajectory Image">
    <img src="doc/image/pipeline.png" width="40%" height="auto" alt="Pipeline Image">
<br>
</div>


## What is Traj-LO
Traj-LO aims to explore the limits of LiDAR sensors in state estimation.

The spatial-temporal movement of LiDAR is parameterized by a simple yet effective continuous-time trajectory, which consists of multiple piecewise linear functions.
By coupling the geometric information from streaming LiDAR points and kinematic constraints from trajectory smoothness, it can work even in scenarios where the motion state exceeds the IMU's measuring range.
Besides, the framework is generalized for different kinds of LiDAR as well as multi-LiDAR systems.
## How to use Traj-LO
In contrast to other LiDAR odometry methods, Traj-LO is a ROS-independent project and is suitable for cross-platform applications. For convenience, we provide a ROSbag data loader that can read public datasets and your own recorded data.

Currently, the released code only supports one LiDAR configuration. For multi-LiDAR support, we will update it as soon as possible.
### Dependency
- Optimization: [Eigen](https://gitlab.com/libeigen/eigen.git), [Sophus](https://github.com/strasdat/Sophus.git)
- GUI: [ImGui](https://github.com/ocornut/imgui), OpenGL, [GLM](https://github.com/g-truc/glm.git)
- DataLoader: TBB, Boost
### Build
Except for the TBB library, we have included other dependencies in the third-party folder. You can install the Traj-LO project by following these steps:
```
git clone --recursive https://github.com/kevin2431/Traj-LO.git
mkdir build 
cd build
cmake .. 
make -j8
```
### Run
The provided ROSbag data loader supports different types of LiDAR, including Livox, Ouster, Hesai, Robosense, and Velodyne. The corresponding configuration files are in the "data" directory.

After modifying the config file for your environment, you can run Traj-LO as follows:
```
./trajlo ../data/config_livox.yaml
```

## Citation

If you use this project for any academic work, please cite our original [paper](https://ieeexplore.ieee.org/document/10387726).

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










