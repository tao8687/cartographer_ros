.. Copyright 2018 The Cartographer Authors

.. Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

..      http://www.apache.org/licenses/LICENSE-2.0

.. Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

=============
深入了解
=============

Cartographer 不仅是一个出色的 SLAM 算法，它还提供了一个功能齐全的实现，带来了许多"额外"功能。
本页列出了一些这些鲜为人知的功能。

更多输入源
==========

如果您有一个里程计数据源(如轮式编码器)在 ``nav_msgs/Odometry`` 话题上发布数据，并想用它来改善 Cartographer 的定位，您可以在 ``.lua`` 配置文件中添加输入:

..  code-block:: lua

    use_odometry = true

消息将在 ``odom`` 话题上接收。

在 ``fix`` 话题上发布 ``sensor_msgs/NavSatFix`` 类型消息的 GPS 可以改善全局 SLAM:

..  code-block:: lua

    use_nav_sat = true

对于在 ``landmark`` 话题上发布 ``cartographer_ros_msgs/LandmarkList`` (`在 cartographer_ros 中定义的消息`_) 类型消息的地标:

..  code-block:: lua

    use_landmarks = true

.. _在 cartographer_ros 中定义的消息: https://github.com/cartographer-project/cartographer_ros/blob/4b39ee68c7a4d518bf8d01a509331e2bc1f514a0/cartographer_ros_msgs/msg/LandmarkList.msg

仅定位模式
=================

如果您已经有了一个满意的地图，想要减少计算量，可以使用 Cartographer 的仅定位模式，它将在现有地图上运行 SLAM 而不会构建新地图。
这可以通过使用 ``-load_state_filename`` 参数运行 ``cartographer_node`` 并在 lua 配置中定义以下内容来启用:

..  code-block:: lua

    TRAJECTORY_BUILDER.pure_localization_trimmer = {
        max_submaps_to_keep = 3,
    }

IMU 标定
===============

在执行全局优化时，Ceres 会尝试改善 IMU 和测距传感器之间的位姿关系。
一个精心选择的采集过程，包含大量回环约束(例如机器人沿直线行驶然后返回)可以提高这些校正的质量，成为可靠的位姿校正来源。
然后您可以将 Cartographer 作为标定过程的一部分，来提高机器人的外部标定质量。

多轨迹 SLAM
=======================

Cartographer 可以同时处理多个机器人并行发送的数据进行 SLAM。
全局 SLAM 能够检测共享路径，并在可能的情况下合并不同机器人构建的地图。
这是通过使用两个 ROS 服务 ``start_trajectory`` 和 ``finish_trajectory`` 来实现的。(有关它们的使用详情，请参考 ROS API 参考文档)

基于 gRPC 的云集成
===========================

Cartographer 基于 Protobuf 消息构建，这使其非常灵活和可互操作。
这种架构的优势之一是它很容易分布在互联网上的多台机器上。
典型的用例是在已知地图上导航的机器人群，它们的 SLAM 算法可以在运行多轨迹 Cartographer 实例的远程强大的集中式定位服务器上运行。

**待办**: 关于如何开始使用 gRPC Cartographer 实例的说明
