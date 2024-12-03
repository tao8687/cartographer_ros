.. Copyright 2016 The Cartographer Authors

.. Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

..      http://www.apache.org/licenses/LICENSE-2.0

.. Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

=========================================
Lua 配置参考文档
=========================================

注意,Cartographer 的 ROS 集成使用 `tf2`_,因此所有坐标系 ID 应该只包含坐标系名称(小写字母加下划线)而不包含前缀或斜杠。有关常用坐标系请参考 `REP 105`_。

注意,在 Cartographer 的 ROS 集成中,话题名称都是以*基础*名称给出的(参见 `ROS Names`_)。这意味着由 Cartographer 节点的用户来决定是否重映射或将它们放入命名空间。

以下是 Cartographer 的 ROS 集成顶层选项,所有这些选项都必须在 Lua 配置文件中指定:

map_frame
  用于发布子地图的 ROS 坐标系 ID,也是位姿的父坐标系,通常为 "map"。

tracking_frame
  被 SLAM 算法跟踪的坐标系的 ROS ID。如果使用 IMU,它应该位于 IMU 的位置,尽管可能会旋转。常见的选择是 "imu_link"。

published_frame
  用作发布位姿的子坐标系的 ROS ID。例如,如果系统的其他部分提供了 "odom" 坐标系,则设为 "odom"。在这种情况下,将发布 *map_frame* 中 "odom" 的位姿。否则,设置为 "base_link" 可能更合适。

odom_frame
  仅在 *provide_odom_frame* 为 true 时使用。用于发布(未闭环的)局部 SLAM 结果的 *published_frame* 和 *map_frame* 之间的坐标系。通常为 "odom"。

provide_odom_frame
  如果启用,将把局部的、未闭环的、连续的位姿作为 *map_frame* 中的 *odom_frame* 发布。

publish_frame_projected_to_2d
  如果启用,发布的位姿将被限制为纯 2D 位姿(无横滚、俯仰或 z 偏移)。这可以防止在 2D 模式下由于位姿外推步骤可能产生的不需要的平面外位姿(例如,如果位姿要作为类似 'base-footprint' 的坐标系发布)。

use_odometry
  如果启用,将订阅话题 "odom" 上的 `nav_msgs/Odometry`_ 消息。在这种情况下必须提供里程计数据,这些信息将被包含在 SLAM 中。

use_nav_sat
  如果启用,将订阅话题 "fix" 上的 `sensor_msgs/NavSatFix`_ 消息。在这种情况下必须提供导航数据,这些信息将被包含在全局 SLAM 中。

use_landmarks
  如果启用,将订阅话题 "landmarks" 上的 `cartographer_ros_msgs/LandmarkList`_ 消息。必须提供地标数据,作为 `cartographer_ros_msgs/LandmarkList`_ 中的 `cartographer_ros_msgs/LandmarkEntry`_。如果提供了 `cartographer_ros_msgs/LandmarkEntry`_ 数据,这些信息将根据 `cartographer_ros_msgs/LandmarkEntry`_ 的 ID 包含在 SLAM 中。`cartographer_ros_msgs/LandmarkList`_ 的采样率应与其他传感器相当。列表可以为空但必须提供,因为 Cartographer 严格按时间顺序处理传感器数据以使地标具有确定性。但是可以将轨迹构建器选项 "collate_landmarks" 设置为 false,以允许非确定性但也非阻塞的方法。

num_laser_scans
  要订阅的激光扫描话题数量。对于单个激光扫描仪,订阅 "scan" 话题上的 `sensor_msgs/LaserScan`_ 消息;对于多个激光扫描仪,订阅 "scan_1"、"scan_2" 等话题。

num_multi_echo_laser_scans
  要订阅的多回波激光扫描话题数量。对于单个激光扫描仪,订阅 "echoes" 话题上的 `sensor_msgs/MultiEchoLaserScan`_ 消息;对于多个激光扫描仪,订阅 "echoes_1"、"echoes_2" 等话题。

num_subdivisions_per_laser_scan
  将每个接收到的(多回波)激光扫描分割成的点云数量。将扫描分割使得可以对扫描仪移动时获取的扫描进行去畸变。有一个相应的轨迹构建器选项可以将分割的扫描累积成用于扫描匹配的点云。

num_point_clouds
  要订阅的点云话题数量。对于单个测距仪,订阅 "points2" 话题上的 `sensor_msgs/PointCloud2`_ 消息;对于多个测距仪,订阅 "points2_1"、"points2_2" 等话题。

lookup_transform_timeout_sec
  查找变换时的超时时间(以秒为单位)。

submap_publish_period_sec
  发布子地图更新的时间间隔(以秒为单位)。

pose_publish_period_sec
  发布位姿更新的时间间隔(以秒为单位)。

trajectory_publish_period_sec
  发布轨迹标记的时间间隔(以秒为单位)。

rangefinder_sampling_ratio
  用于构建子地图的测距数据采样比例。

odometry_sampling_ratio
  用于局部 SLAM 的里程计数据采样比例。

fixed_frame_pose_sampling_ratio
  用于局部 SLAM 的固定帧位姿数据采样比例。

imu_sampling_ratio
  用于局部 SLAM 的 IMU 数据采样比例。

landmarks_sampling_ratio

.. _REP 105: http://www.ros.org/reps/rep-0105.html
.. _ROS Names: http://wiki.ros.org/Names
.. _geometry_msgs/PoseStamped: http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
.. _nav_msgs/OccupancyGrid: http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
.. _nav_msgs/Odometry: http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html
.. _sensor_msgs/LaserScan: http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
.. _sensor_msgs/MultiEchoLaserScan: http://docs.ros.org/api/sensor_msgs/html/msg/MultiEchoLaserScan.html
.. _sensor_msgs/PointCloud2: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
.. _sensor_msgs/NavSatFix: http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html
.. _cartographer_ros_msgs/LandmarkList: https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros_msgs/msg/LandmarkList.msg
.. _cartographer_ros_msgs/LandmarkEntry: https://github.com/cartographer-project/cartographer_ros/blob/4b39ee68c7a4d518bf8d01a509331e2bc1f514a0/cartographer_ros_msgs/msg/LandmarkEntry.msg
.. _tf2: http://wiki.ros.org/tf2
