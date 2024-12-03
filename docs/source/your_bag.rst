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

========================================
在您自己的数据包上运行
========================================

现在您已经在一些提供的数据包上运行了 Cartographer ROS,您可以继续让 Cartographer 处理您自己的数据。
找到一个您想用于 SLAM 的 ``.bag`` 记录,然后按照本教程进行操作。

.. warning:: 当您想要运行 cartographer_ros 时,可能需要先运行 ``source install_isolated/setup.bash`` 来设置 ROS 环境(如果您使用的是 zsh shell,则将 bash 替换为 zsh)

要求
============

以下是对数据包的要求:

- ``tf`` 消息
- ``sensor_msgs/LaserScan`` 或 ``sensor_msgs/MultiEchoLaserScan`` 或 ``sensor_msgs/PointCloud2`` 消息
- 对于3D SLAM: 还需要 ``sensor_msgs/Imu`` 消息

下面的 ROS 节点验证您的数据包是否满足这些要求:

.. code-block:: bash

    roslaunch cartographer_ros offline_backpack_2d.launch bag_filenames:=${HOME}/Downloads/your_bag.bag

或者对于3D SLAM:

.. code-block:: bash

    roslaunch cartographer_ros offline_backpack_3d.launch bag_filenames:=${HOME}/Downloads/your_bag.bag

验证您的数据包
=================

Cartographer ROS 提供了一个名为 ``cartographer_rosbag_validate`` 的工具来自动分析您数据包中的数据。
在尝试为不正确的数据调优 Cartographer 之前,运行此工具通常是个好主意。

它受益于 Cartographer 作者的经验,可以检测到数据包中常见的各种错误。
例如,如果检测到 ``sensor_msgs/Imu`` 话题,该工具将确保重力向量没有从 IMU 测量中移除,因为 Cartographer 使用重力范数来确定地面的方向。

该工具还可以提供如何改善数据质量的建议。
例如,对于 Velodyne 激光雷达,建议每个传感器发送的 UDP 数据包对应一个 ``sensor_msgs/PointCloud2`` 消息,而不是每转一圈对应一个消息。
有了这种粒度,Cartographer 就能够消除由机器人运动引起的点云变形,从而获得更好的重建效果。

如果您已经设置了 Cartographer ROS 环境,您可以像这样简单地运行该工具:

..  code-block:: bash

    cartographer_rosbag_validate -bag_filename your_bag.bag

创建 .lua 配置文件
===========================

Cartographer 具有高度的灵活性,可以配置为在各种机器人上工作。
机器人配置从必须由 Lua 脚本定义的 ``options`` 数据结构中读取。
示例配置定义在 ``src/cartographer_ros/cartographer_ros/configuration_files`` 中,并安装在 ``install_isolated/share/cartographer_ros/configuration_files/`` 中。

.. note:: 理想情况下,.lua 配置应该是特定于机器人的,而不是特定于数据包的。

您可以从一个示例开始复制,然后根据自己的需求进行调整。如果您想使用3D SLAM:

..  code-block:: bash

    cp install_isolated/share/cartographer_ros/configuration_files/backpack_3d.lua install_isolated/share/cartographer_ros/configuration_files/my_robot.lua
 
如果您想使用2D SLAM:

..  code-block:: bash

    cp install_isolated/share/cartographer_ros/configuration_files/backpack_2d.lua install_isolated/share/cartographer_ros/configuration_files/my_robot.lua

然后您可以编辑 ``my_robot.lua`` 以适应您的机器人的需求。
``options`` 块中定义的值决定了 Cartographer ROS 前端应该如何与您的数据包交互。
在 ``options`` 段落之后定义的值用于调整 Cartographer 的内部工作,我们暂时忽略这些。

.. seealso:: `Cartographer ROS 配置值的参考文档`_ 和 `Cartographer 配置值的参考文档`_。

.. _Cartographer ROS 配置值的参考文档: https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html

.. _Cartographer 配置值的参考文档: https://google-cartographer.readthedocs.io/en/latest/configuration.html

在需要调整的值中,您可能需要在 ``map_frame``、``tracking_frame``、``published_frame`` 和 ``odom_frame`` 中提供环境和机器人的 TF 帧 ID。

.. note:: 您可以通过数据包中的 ``/tf`` 话题分发机器人的 TF 树,或在 ``.urdf`` 机器人定义中定义它。

.. warning:: 您应该信任您的位姿! 机器人与 IMU 或激光雷达之间链接的小偏移可能导致不连贯的地图重建。Cartographer 通常可以纠正小的位姿错误,但不是所有错误!

您需要定义的其他值与您想要使用的传感器的数量和类型有关。

- ``num_laser_scans``: 您将使用的 ``sensor_msgs/LaserScan`` 话题数量。
- ``num_multi_echo_laser_scans``: 您将使用的 ``sensor_msgs/MultiEchoLaserScan`` 话题数量。
- ``num_point_clouds``: 您将使用的 ``sensor_msgs/PointCloud2`` 话题数量。

您还可以使用 ``use_landmarks`` 和 ``use_nav_sat`` 启用地标和 GPS 作为额外的定位源。``options`` 块中的其余变量通常应保持不变。

.. note:: 即使您使用2D SLAM,地标也是3D对象,由于它们的第三维度,如果仅在2D平面上查看可能会误导您。

然而,有一个全局变量您绝对需要根据数据包的需求进行调整:``TRAJECTORY_BUILDER_3D.num_accumulated_range_data`` 或 ``TRAJECTORY_BUILDER_2D.num_accumulated_range_data``。
这个变量定义了构建完整扫描(通常是一整圈)所需的消息数量。
如果您遵循 ``cartographer_rosbag_validate`` 的建议并每次扫描使用100个 ROS 消息,您可以将此变量设置为100。
如果您有两个测距传感器(例如,两个激光雷达)一次性提供它们的完整扫描,您应该将此变量设置为2。

创建 .launch 文件
===================

启动文件定义了 ROS 节点如何与您的数据包交互。
您可以从示例启动文件开始:

..  code-block:: bash

    cp install_isolated/share/cartographer_ros/launch/offline_backpack_3d.launch \
       install_isolated/share/cartographer_ros/launch/my_robot.launch

或者对于2D SLAM:

..  code-block:: bash

    cp install_isolated/share/cartographer_ros/launch/offline_backpack_2d.launch \
       install_isolated/share/cartographer_ros/launch/my_robot.launch

您需要调整启动文件中的以下内容:

1. 将 ``configuration_directory`` 和 ``configuration_basename`` 参数设置为指向您的 ``.lua`` 配置文件。

2. 如果您使用 ``.urdf`` 文件来描述您的机器人,请将 ``urdf_filename`` 参数设置为指向该文件。

3. 如果您的数据包中的传感器话题名称与默认值不同,您需要重新映射它们:

   - 对于 ``sensor_msgs/LaserScan`` 消息,默认话题名称是 ``scan`` 或 ``scan_1``、``scan_2`` 等。
   - 对于 ``sensor_msgs/MultiEchoLaserScan`` 消息,默认话题名称是 ``echoes`` 或 ``echoes_1``、``echoes_2`` 等。
   - 对于 ``sensor_msgs/PointCloud2`` 消息,默认话题名称是 ``points2`` 或 ``points2_1``、``points2_2`` 等。
   - 对于 ``sensor_msgs/Imu`` 消息,默认话题名称是 ``imu``。

4. 如果您使用多个传感器,请确保它们的话题名称按顺序编号。

运行 Cartographer
===================

一旦您完成了所有配置,您就可以使用以下命令运行 Cartographer:

..  code-block:: bash

    roslaunch cartographer_ros my_robot.launch bag_filenames:=${HOME}/Downloads/your_bag.bag

如果一切设置正确,您应该看到 RViz 窗口显示正在构建的地图。

.. note:: 如果您看到错误,请检查 ROS 控制台输出以获取详细信息。最常见的问题是话题名称不匹配或传感器数据格式不正确。

调优
=====

一旦基本设置工作正常,您可能想要调优配置以获得更好的结果。
请参阅 :doc:`tuning` 获取详细指导。
