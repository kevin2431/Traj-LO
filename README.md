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

The spatial-temporal movement of LiDAR is parameterized by a simple yet effective continuous-time trajectory, which consists of multiple piecewise linear functions. 
By coupling the geometric information from streaming LiDAR points and kinematic constraints from trajectory smoothness, it can work even in scenarios where the motion state exceeds the IMU's measuring range.
Besides, the framework is generalized for different kinds of LiDAR as well as multi-LiDAR systems.

## News: Traj-LO is accepted to [RA-L](https://ieeexplore.ieee.org/document/10387726)
**2024.3.1** Sorry guys. Apologies for the delay in releasing the code, as I am currently managing numerous tasks before my graduation. I will endeavor to refactor the code in this March. Thank you for your attention to our work. 

For those interested in multi-sensor fusion, particularly in multi-LiDAR and multi-IMU systems, I recommend checking out my latest work, [Traj-LIO](https://arxiv.org/abs/2402.09189), which is a resilient state estimator through sparse Gaussian Processes.









**Video demo** [Youtube](https://www.youtube.com/watch?v=hbtKzElYKkQ) [Bilibili](https://www.bilibili.com/video/BV1Ky4y1F7uT)
<div align="center">
    <a href="https://www.youtube.com/watch?v=hbtKzElYKkQ">
        <img src="http://img.youtube.com/vi/hbtKzElYKkQ/0.jpg" width="640" height="480" alt="Video Cover">
    </a>
</div>





