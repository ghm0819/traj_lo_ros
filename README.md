# traj_lo_ros
ROS version of Traj_LO (state estimation using only LiDAR sensors)

## What is Traj-LO
**Traj-LO aims to explore the limits of state estimation using only LiDAR sensors.**

Nowadays, LO approaches heavily rely on IMU data for accurate state estimation but neglect the true capabilities of LiDAR sensors. Through the continuous-time perspective, Traj-LO matches the performance of state-of-the-art LIO methods in diverse scenarios.

The spatial-temporal movement of LiDAR is parameterized by a simple yet effective continuous-time trajectory, which consists of multiple piecewise linear functions.
By coupling the geometric information from streaming LiDAR points and kinematic constraints from trajectory smoothness, it can work even in scenarios where the motion state exceeds the IMU's measuring range.
Besides, the framework is generalized for different kinds of LiDAR as well as multi-LiDAR systems.


## Quick Start

The project is the ROS version of Traj_LO, which has been tested on 20.04 (ROS Noetic), and it can also be tested in other ROS versions (such as melodic with Ubuntu 18.04 or earlier versions). In the following, we will take the ROS Noetic version as an example. 

1. First, create and initialize a ROS workspace, and run the following commands to clone this repo and build it:

   ```
   $ mkdir -p ~/catkin_traj/src
   $ cd ~/catkin_traj/src
   $ catkin_init_workspace
   $ git clone https://github.com/ghm0819/traj_lo_ros.git
   $ cd ..
   $ source ~/ws_livox/devel/setup.bash
   $ catkin_make
   ```

   If you encounter a problem with missing packages during the installation process, please install the corresponding package based on the specific compilation error.


2. Note that, the lidar topic and config information of the test dataset should be modified in `trajectory_tracking.launch`.

   ```
   $ roslaunch traj_lo trajectory_tracking.launch
   ```

   And open a new terminal and play the bag (only LiDAR data).

   ```
   rosbag play --clock bag_name.bag --topics /livox/lidar
   ```

   Finally, you can view the lidar odometry through RVIZ.


3. This lidar odometry cannot achieve real-time performance when the point cloud contains a large number of points. Better real-time performance can be achieved with Livox-series lidars (since each frame contains fewer points).


## Citation

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

## Acknowledgement
Thanks for the pioneering work [Traj_LO](https://github.com/kevin2431/Traj-LO).
