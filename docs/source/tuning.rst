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

.. cartographer SHA: aba4575d937df4c9697f61529200c084f2562584
.. cartographer_ros SHA: 99c23b6ac7874f7974e9ed808ace841da6f2c8b0
.. TODO(hrapp): mention insert_free_space somewhere

调优指南
=========

Cartographer 的调优是一个复杂的任务,因为许多参数都会相互影响。本指南试图为调优过程提供一个系统的方法。

基本原则
===========

- 在开始调优之前,请确保您已经阅读并理解了 :doc:`algo_walkthrough`。
- 调优应该从局部 SLAM 开始,然后再调整全局 SLAM。
- 调优是一个迭代过程:更改一个参数可能需要重新调整先前优化的参数。

局部 SLAM 调优
=================

1. 首先禁用全局 SLAM:

   .. code-block:: lua

       POSE_GRAPH.optimize_every_n_nodes = 0

2. 如果您使用 IMU 数据:

   - 确保 IMU 数据质量良好。
   - 检查 IMU 的外参标定是否正确。
   - 调整 ``TRAJECTORY_BUILDER_nD.imu_gravity_time_constant`` 以平衡噪声过滤和响应速度。

3. 调整传感器数据过滤:

   - 设置适当的 ``min_range`` 和 ``max_range``。
   - 如果点云太密集,增加 ``voxel_filter_size``。
   - 调整 ``num_accumulated_range_data`` 以平衡实时性和扫描质量。

4. 调整扫描匹配器:

   - 如果您不信任其他传感器,启用 ``use_online_correlative_scan_matching``。
   - 调整 ``ceres_scan_matcher`` 的权重:
     
     * ``translation_weight`` - 控制平移代价
     * ``rotation_weight`` - 控制旋转代价
     * ``occupied_space_weight`` - 控制占用空间匹配的重要性

5. 优化子地图设置:

   - 调整 ``submaps.num_range_data`` 以平衡局部一致性和漂移。
   - 选择合适的 ``grid_options_2d.resolution``。

全局 SLAM 调优
=================

1. 启用全局 SLAM 并设置合适的优化频率:

   .. code-block:: lua

       POSE_GRAPH.optimize_every_n_nodes = 90

2. 调整约束生成:

   - ``constraint_builder.sampling_ratio`` - 控制用于回环检测的节点数量
   - ``constraint_builder.min_score`` - 控制约束的质量阈值
   - ``constraint_builder.global_localization_min_score`` - 控制全局定位的质量要求

3. 调整优化问题:

   - 调整各种权重以平衡不同来源的信息:
     
     * ``optimization_problem.local_slam_pose_weight``
     * ``optimization_problem.odometry_weight``
     * ``optimization_problem.constraint_weight``

特殊用例
=========

低延迟模式
-----------

对于需要实时性能的应用,可以:

1. 减少计算负载:

   - 降低 ``num_background_threads``
   - 增加 ``voxel_filter_size``
   - 减少 ``num_range_data``

2. 调整全局优化:

   - 增加 ``optimize_every_n_nodes``
   - 减少 ``constraint_builder.sampling_ratio``
   - 增加 ``constraint_builder.min_score``

纯定位模式
-----------

在已有地图中进行定位时:

1. 启用纯定位模式:

   .. code-block:: lua

       TRAJECTORY_BUILDER.pure_localization = true

2. 调整优化频率:

   - 减小 ``optimize_every_n_nodes``
   - 减小 ``global_sampling_ratio``
   - 减小 ``constraint_builder.sampling_ratio``

3. 确保子地图分辨率与已有地图匹配。

仍有问题?
----------

如果您仍然无法使 Cartographer 在您的数据上可靠工作,您可以:

1. 查看已关闭的 GitHub issues,寻找类似问题的解决方案。
2. 创建新的 GitHub issue 寻求帮助,请确保:
   
   - 包含 ``rosbag_validate`` 的结果
   - 提供您的配置文件
   - 提供可重现问题的 .bag 文件

.. note::

   开发人员很乐意提供帮助,但他们只能在您提供完整信息的情况下提供有效帮助。
