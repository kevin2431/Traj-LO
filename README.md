<div align="center">
    <h1>Traj-LO</h1>
    <a href=https://youtu.be/hbtKzElYKkQ?si=ZlqvtUVhhJbAju0S>Video</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="">Paper</a>
</div>

![traj](doc/image/trajectory.png)
**Traj-LO is a **LiDAR-only** Odometry that focus on improving accuracy through **continuous-time** perespective.
The spatialtemporal movement of LiDAR is parameterized by a simple yet effective continuous-time trajectory, which is consisted of multiple piecewise linear functions.** 

![traj](doc/image/pipeline.png)

By coupling the gemotric information from streaming LiDAR points and kinematic constraints from trajectory smoothness, it can work even in scenarios where the motion state exceeds the IMU's measuring range.
Besides, the framework is generalized for different kinds of LiDAR as well as multi-LiDAR systems.





