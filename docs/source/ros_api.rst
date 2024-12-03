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

===============================
ROS API 参考文档
===============================

.. image:: nodes_graph_demo_2d.jpg

Cartographer 节点
=================

`cartographer_node`_ 是用于在线实时 SLAM 的节点。

.. _cartographer_node: https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros/cartographer_ros/node_main.cc

命令行参数
------------------

使用 ``--help`` 参数运行节点可以查看所有可用选项。

订阅的话题
-----------------

以下测距数据话题是互斥的。至少需要一个测距数据源。

scan (`sensor_msgs/LaserScan`_)
  支持2D和3D(例如使用轴向旋转的平面激光扫描仪)。
  如果在 :doc:`configuration` 中将 *num_laser_scans* 设置为1,此话题将作为 SLAM 的输入。
  如果 *num_laser_scans* 大于1,则多个编号的扫描话题(即 scan_1、scan_2、scan_3...直到包括 *num_laser_scans*)将作为 SLAM 的输入。

echoes (`sensor_msgs/MultiEchoLaserScan`_)
  支持2D和3D(例如使用轴向旋转的平面激光扫描仪)。
  如果在 :doc:`configuration` 中将 *num_multi_echo_laser_scans* 设置为1,此话题将作为 SLAM 的输入。
  仅使用第一次回波。如果 *num_multi_echo_laser_scans* 大于1,则多个编号的回波话题(即 echoes_1、echoes_2、echoes_3...直到包括 *num_multi_echo_laser_scans*)将作为 SLAM 的输入。

points2 (`sensor_msgs/PointCloud2`_)
  如果在 :doc:`configuration` 中将 *num_point_clouds* 设置为1,此话题将作为 SLAM 的输入。
  如果 *num_point_clouds* 大于1,则多个编号的 points2 话题(即 points2_1、points2_2、points2_3...直到包括 *num_point_clouds*)将作为 SLAM 的输入。

还可以提供以下额外的传感器数据话题:

imu (`sensor_msgs/Imu`_)
  支持2D(可选)和3D(必需)。此话题将作为 SLAM 的输入。

odom (`nav_msgs/Odometry`_)
  支持2D(可选)和3D(可选)。如果在 :doc:`configuration` 中启用了 *use_odometry*,此话题将作为 SLAM 的输入。

.. TODO: 添加 NavSatFix? Landmarks?

发布的话题
----------------

scan_matched_points2 (`sensor_msgs/PointCloud2`_)
  用于扫描与子地图匹配的点云。根据 :doc:`configuration` 的设置,此点云可能经过过滤和投影。

submap_list (`cartographer_ros_msgs/SubmapList`_)
  所有子地图的列表,包括每个子地图在所有轨迹中的位姿和最新版本号。

tracked_pose (`geometry_msgs/PoseStamped`_)
  仅在参数 ``publish_tracked_pose`` 设置为 ``true`` 时发布。
  跟踪帧相对于地图帧的位姿。

服务
--------

所有服务响应都包含一个 ``StatusResponse``,它包含一个 ``code`` 和一个 ``message`` 字段。
为了保持一致性,整数 ``code`` 等同于 `gRPC`_ API 中使用的状态码。

.. _gRPC: https://developers.google.com/maps-booking/reference/grpc-api/status_codes

submap_query (`cartographer_ros_msgs/SubmapQuery`_)
  获取请求的子地图。

start_trajectory (`cartographer_ros_msgs/StartTrajectory`_)
  使用默认传感器话题和提供的配置启动轨迹。
  可以选择指定初始位姿。返回分配的轨迹 ID。

trajectory_query (`cartographer_ros_msgs/TrajectoryQuery`_)
  从位姿图返回轨迹数据。

finish_trajectory (`cartographer_ros_msgs/FinishTrajectory`_)
  通过运行最终优化来完成给定 `trajectory_id` 的轨迹。

write_state (`cartographer_ros_msgs/WriteState`_)
  将当前内部状态写入磁盘的 `filename`。文件通常会保存在 `~/.ros` 或设置的 `ROS_HOME` 中。
  此文件可以作为 `assets_writer_main` 的输入,用于生成概率网格、X射线或 PLY 文件等资产。

get_trajectory_states (`cartographer_ros_msgs/GetTrajectoryStates`_)
  返回轨迹的 ID 和状态。
  例如,这对于从单独的节点观察 Cartographer 的状态很有用。

read_metrics (`cartographer_ros_msgs/ReadMetrics`_)
  返回 Cartographer 所有内部指标的最新值。
  运行时指标的收集是可选的,必须在节点中使用 ``--collect_metrics`` 命令行标志激活。

必需的 tf 变换
----------------------

.. image:: frames_demo_2d.jpg

必须提供从所有传入传感器数据帧到 :doc:`配置的 <configuration>` *tracking_frame* 和 *published_frame* 的变换。
通常,这些由 `robot_state_publisher` 或 `static_transform_publisher` 定期发布。

提供的 tf 变换
----------------------

除非参数 ``publish_to_tf`` 设置为 ``false``,否则将提供 :doc:`配置的 <configuration>` *map_frame* 和 *published_frame* 之间的变换。

如果在 :doc:`配置 <configuration>` 中启用了 *provide_odom_frame*,还将提供 :doc:`配置的 <configuration>` *odom_frame* 和 *published_frame* 之间的连续(即不受回环闭合影响)变换。

离线节点
============

`offline_node`_ 是对传感器数据包进行 SLAM 的最快方式。
它不监听任何话题,而是从命令行提供的一组数据包中读取 TF 和传感器数据。
它还会随着传感器数据的推进发布时钟,即替代 ``rosbag play``。
在其他所有方面,它的行为与 ``cartographer_node`` 相同。
每个数据包将在最终状态中成为一个单独的轨迹。
一旦处理完所有数据,它就会写出最终的 Cartographer 状态并退出。

.. _offline_node: https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros/cartographer_ros/offline_node_main.cc


发布的话题
----------------

除了在线节点发布的话题外,此节点还发布:

~bagfile_progress (`cartographer_ros_msgs/BagfileProgress`_)
  包文件处理进度,包括有关当前正在处理的包的详细信息,这些信息将按预定义的间隔发布,可以使用 ``~bagfile_progress_pub_interval`` ROS 参数指定。

.. _cartographer_ros_msgs/BagfileProgress: https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros_msgs/msg/BagfileProgress.msg

参数
----------

~bagfile_progress_pub_interval (double, default=10.0):
  发布包文件处理进度的时间间隔,单位为秒。


Occupancy grid 节点
===================

`occupancy_grid_node`_ 监听 SLAM 发布的子地图,构建 ROS 占用网格,并发布它。
此工具对于让旧节点继续工作,直到新的导航堆栈可以处理 Cartographer 的子地图,非常有用。
生成地图是昂贵且缓慢的,因此地图更新是按秒进行的。
您可以使用命令行选项选择性地包括/排除静态或活动轨迹中的子地图。
使用 ``--help`` 参数运行节点可以查看这些选项。

.. _occupancy_grid_node: https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros/cartographer_ros/occupancy_grid_node_main.cc

订阅的话题
-----------------

它订阅 Cartographer 的 ``submap_list`` 话题。

发布的话题
----------------

map (`nav_msgs/OccupancyGrid`_)
  如果订阅,节点将连续计算并发布地图。更新间隔将随着地图大小的增加而增加。对于更快的更新,请使用子地图 API。


Pbstream Map Publisher 节点
===========================

`pbstream_map_publisher`_ 是一个简单的节点,它创建一个静态占用网格,从序列化的 Cartographer 状态(pbstream 格式)。
它是一个高效的替代方案,如果不需要实时更新,则可以替代占用网格节点。

.. _pbstream_map_publisher: https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros/cartographer_ros/pbstream_map_publisher_main.cc

订阅的话题
-----------------

None.

发布的话题
----------------

map (`nav_msgs/OccupancyGrid`_)
  发布的占用网格话题是锁定的。
