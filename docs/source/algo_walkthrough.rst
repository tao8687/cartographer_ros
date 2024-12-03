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

算法原理及调优指南
================================

Cartographer 是一个复杂的系统,调优它需要深入理解其内部工作原理。本文档试图直观地介绍 Cartographer 使用的各个子系统及其配置参数。
如果您想了解比入门更深入的内容,应该参考 Cartographer 论文。
该论文主要描述了2D SLAM,但严格定义了此处描述的大多数概念。
这些概念通常也适用于3D场景。

W. Hess, D. Kohler, H. Rapp, and D. Andor,
`Real-Time Loop Closure in 2D LIDAR SLAM`_, in
*Robotics and Automation (ICRA), 2016 IEEE International Conference on*.
IEEE, 2016. pp. 1271–1278.

.. _Real-Time Loop Closure in 2D LIDAR SLAM: https://research.google.com/pubs/pub45466.html

概述
--------

.. image:: https://raw.githubusercontent.com/cartographer-project/cartographer/master/docs/source/high_level_system_overview.png
     :target: https://github.com/cartographer-project/cartographer/blob/master/docs/source/high_level_system_overview.png

Cartographer 可以看作两个独立但相关的子系统。
第一个是**局部 SLAM**(有时也称为**前端**或局部轨迹构建器)。
它的工作是构建一系列的**子地图**。
每个子地图在局部都是一致的,但我们接受局部 SLAM 随时间的漂移。
大多数局部 SLAM 选项可以在 2D 的 `install_isolated/share/cartographer/configuration_files/trajectory_builder_2d.lua`_ 和 3D 的 `install_isolated/share/cartographer/configuration_files/trajectory_builder_3d.lua`_ 中找到。(本文档后续将用 `TRAJECTORY_BUILDER_nD` 表示通用选项)

.. _install_isolated/share/cartographer/configuration_files/trajectory_builder_2d.lua: https://github.com/cartographer-project/cartographer/blob/df337194e21f98f8c7b0b88dab33f878066d4b56/configuration_files/trajectory_builder_2d.lua
.. _install_isolated/share/cartographer/configuration_files/trajectory_builder_3d.lua: https://github.com/cartographer-project/cartographer/blob/df337194e21f98f8c7b0b88dab33f878066d4b56/configuration_files/trajectory_builder_3d.lua

另一个子系统是**全局 SLAM**(有时称为**后端**)。
它在后台线程中运行,其主要工作是寻找**回环检测约束**。
它通过将**扫描**(收集在**节点**中)与子地图进行扫描匹配来实现这一点。
它还结合其他传感器数据以获得更高层次的视图并识别最一致的全局解决方案。
在3D中,它还试图找到重力方向。
其大部分选项可以在 `install_isolated/share/cartographer/configuration_files/pose_graph.lua`_ 中找到。

.. _install_isolated/share/cartographer/configuration_files/pose_graph.lua: https://github.com/cartographer-project/cartographer/blob/df337194e21f98f8c7b0b88dab33f878066d4b56/configuration_files/pose_graph.lua

在更高的抽象层面上,局部 SLAM 的工作是生成好的子地图,而全局 SLAM 的工作是将它们最一致地连接在一起。

输入
-----

测距传感器(例如:激光雷达)在多个方向提供深度信息。
然而,某些测量对于 SLAM 来说是无关的。
如果传感器被灰尘部分遮挡或指向机器人的某个部分,一些测量距离对于 SLAM 来说可以被视为噪声。
另一方面,一些最远的测量也可能来自不需要的源(反射、传感器噪声),对于 SLAM 也是无关的。
为了解决这些问题,Cartographer 首先应用带通滤波器,只保留在特定最小和最大范围之间的距离值。
这些最小和最大值应根据您的机器人和传感器的规格来选择。

.. code-block:: lua

    TRAJECTORY_BUILDER_nD.min_range
    TRAJECTORY_BUILDER_nD.max_range

.. note::

    在2D中,Cartographer 将超过 max_range 的范围替换为 ``TRAJECTORY_BUILDER_2D.missing_data_ray_length``。它还提供 ``max_z`` 和 ``min_z`` 值来将3D点云过滤成2D切片。

.. note::

    在 Cartographer 配置文件中,所有距离都以米为单位定义

距离是在机器人实际移动时的一段时间内测量的。
然而,距离是由传感器"批量"在大型 ROS 消息中传递的。
Cartographer 可以独立考虑每个消息的时间戳,以考虑机器人运动造成的变形。
Cartographer 获得测量的频率越高,就越能更好地对测量进行去畸变,组装成一个可以瞬时捕获的连贯扫描。
因此,强烈建议在每次扫描(可以与另一次扫描匹配的一组测距数据)中提供尽可能多的测距数据(ROS 消息)。

.. code-block:: lua

    TRAJECTORY_BUILDER_nD.num_accumulated_range_data

测距数据通常是从机器人上的单点以多个角度测量的。
这意味着近距离表面(例如地面)经常被击中并提供大量点。
相反,远处物体被击中的次数较少,提供的点也较少。
为了减少点处理的计算量,我们通常需要对点云进行降采样。
然而,简单的随机采样会从已经测量密度较低的区域移除点,而高密度区域仍然会有超过需要的点。
为了解决这个密度问题,我们可以使用体素滤波器,将原始点降采样到固定大小的立方体中,只保留每个立方体的质心。

较小的立方体尺寸会导致更密集的数据表示,造成更多的计算。
较大的立方体尺寸会导致数据损失,但会快得多。

.. code-block:: lua

    TRAJECTORY_BUILDER_nD.voxel_filter_size

在应用固定大小的体素滤波器后,Cartographer 还应用了**自适应体素滤波器**。
该滤波器尝试确定最优体素大小(在最大长度下),以实现目标点数。
在 3D 中,两个自适应体素滤波器用于生成高分辨率和低分辨率点云,它们的用法将在 :ref:`local-slam` 中澄清。

.. code-block:: lua

    TRAJECTORY_BUILDER_nD.*adaptive_voxel_filter.max_length
    TRAJECTORY_BUILDER_nD.*adaptive_voxel_filter.min_num_points

惯性测量单元可以作为 SLAM 的有用信息源,因为它提供了准确的地面方向(因此,地面)和噪声但总体上的机器人旋转指示。
为了过滤掉 IMU 噪声,重力被观察一段时间。
如果你使用 2D SLAM,你可以选择是否让 Cartographer 使用 IMU,因为范围数据可以在没有其他信息源的情况下实时处理。
对于 3D SLAM,你需要提供 IMU,因为它被用作扫描方向的初始猜测,大大减少了扫描匹配的复杂性。


.. code-block:: lua

    TRAJECTORY_BUILDER_2D.use_imu_data
    TRAJECTORY_BUILDER_nD.imu_gravity_time_constant

.. note::

   在 Cartographer 配置文件中,每个时间值都以秒为单位定义。

.. _local-slam:

局部 SLAM
----------

一扫描被组装和过滤掉多个范围数据,它就准备好了局部 SLAM 算法。
局部 SLAM 通过**扫描匹配**将新扫描插入其当前子地图构建中,使用来自**位姿外推器**的初始猜测。
位姿外推器的想法是使用其他传感器的传感器数据来预测下一个扫描应该插入子地图的位置。

有两种扫描匹配策略可用:

- ``CeresScanMatcher`` 将初始猜测作为先验,找到扫描匹配最适合子地图的最佳位置。
  它通过插值子地图和子像素对齐扫描来实现这一点。
  这是快速的,但无法修复误差,这些误差显著大于子地图的分辨率。
  如果你的传感器设置和时间合理,使用 ``CeresScanMatcher`` 通常是最佳选择。
- ``RealTimeCorrelativeScanMatcher`` 可以启用,如果你没有其他传感器或你不信任它们。
  它使用与回环检测(稍后描述)类似的方匹配当前子地图中的扫描。
  最佳匹配用于 ``CeresScanMatcher`` 的先验。
  这个扫描匹配器非常昂贵,将基本上覆盖其他传感器但范围检测器的信号,但它对特征丰富的环境非常稳健。

无论哪种方式,``CeresScanMatcher`` 都可以配置为给每个输入一个特定的权重。
权重是一个信任度量,可以看作是静态协方差。
权重参数的单位是无量纲量,不能在每个之比较。
数据源的权重越大,Cartographer 在扫描匹配时越重视该数据源。
数据源包括占用空间(扫描点)、来自位姿外推器(或 ``RealTimeCorrelativeScanMatcher``)的平移和旋转。

.. code-block:: lua

    TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight
    TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0
    TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1
    TRAJECTORY_BUILDER_nD.ceres_scan_matcher.translation_weight
    TRAJECTORY_BUILDER_nD.ceres_scan_matcher.rotation_weight

.. note::

   在 3D 中,``occupied_space_weight_0`` 和 ``occupied_space_weight_1`` 参数与高分辨率和低分辨率过滤点云相关。

``CeresScanMatcher`` 的名称来自 `Ceres Solver`_,一个由 Google 开发的库,用于解决非线性最小二乘问题。
扫描匹配问题被建模为最小化此类问题,其中两个扫描之间的 **运动** (变换矩阵)是确定参数。
Ceres 使用下降算法优化运动,对于给定的迭代次数。
Ceres 可以配置为适应您的需求调整收敛速度。

.. _Ceres Solver: http://ceres-solver.org/

.. code-block:: lua

    TRAJECTORY_BUILDER_nD.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps
    TRAJECTORY_BUILDER_nD.ceres_scan_matcher.ceres_solver_options.max_num_iterations
    TRAJECTORY_BUILDER_nD.ceres_scan_matcher.ceres_solver_options.num_threads

``RealTimeCorrelativeScanMatcher`` 可以根据你对传感器信任度来切换。
它通过在 **搜索窗口** 中搜索相似扫描来工作,该窗口由最大距离半径和最大角度半径定义。
当与搜索窗口中的扫描进行扫描匹配时,可以为平移和旋转分量选择不同的权重。
你可以调整这些权重,例如,如果你知道你的机器人不经常旋转。

.. code-block:: lua

    TRAJECTORY_BUILDER_nD.use_online_correlative_scan_matching
    TRAJECTORY_BUILDER_nD.real_time_correlative_scan_matcher.linear_search_window
    TRAJECTORY_BUILDER_nD.real_time_correlative_scan_matcher.angular_search_window
    TRAJECTORY_BUILDER_nD.real_time_correlative_scan_matcher.translation_delta_cost_weight
    TRAJECTORY_BUILDER_nD.real_time_correlative_scan_matcher.rotation_delta_cost_weight

为了避免在子地图中插入太多扫描,一旦通过扫描匹配器找到两个扫描之间的运动,它就会通过**运动滤波器**。
如果运动不是被认为是显著的,扫描将被丢弃。
只有在运动超过一定距离、角度或时间阈值时,扫描才会插入当前子地图。

.. code-block:: lua

    TRAJECTORY_BUILDER_nD.motion_filter.max_time_seconds
    TRAJECTORY_BUILDER_nD.motion_filter.max_distance_meters
    TRAJECTORY_BUILDER_nD.motion_filter.max_angle_radians

子地图被认为完成时,局部 SLAM 已经接收了一定数量的范围数据。
局部 SLAM 随时间漂移,全局 SLAM 用于修复这种漂移。
子地图必须足够小,以便子地图内的漂移低于分辨率,以便它们在局部是正确的。
另一方面,它们应该足够大,以便在回环检测时正确工作。

.. code-block:: lua

    TRAJECTORY_BUILDER_nD.submaps.num_range_data

子地图可以用几种不同的数据结构存储其范围数据:
最广泛使用的表现形式是概率网格。
然而,在 2D 中,也可以选择使用截断符号距离场 (TSDF)。

.. code-block:: lua

    TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type

概率网格将空间切成 2D 或 3D 表格,每个单元格都有固定大小并包含被占据的概率。
概率根据 "*hits*" (测量范围数据)和 "*misses*" (传感器和测量点之间的自由空间)更新。
*hits* 和 *misses* 可以在占用概率计算中具有不同的权重,从而对占用或自由空间测量值给予更多或更少的信任。

.. code-block:: lua

    TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability
    TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability
    TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability
    TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.miss_probability

在 2D 中,每个子地图只存储一个概率网格。
在 3D 中,为了扫描匹配性能原因,使用了两个 *混合* 概率网格。
(术语 "混合"仅指内部树状数据表示,并抽象为用户)

- 低分辨率混合网格用于远距离测量
- 高分辨率混合网格用于近距离测量

扫描匹配从将低分辨率点云的远点与低分辨率混合网格对齐开始,然后通过将高分辨率点的近点与高分辨率混合网格对齐来细化位置。

.. code-block:: lua

    TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution
    TRAJECTORY_BUILDER_3D.submaps.high_resolution
    TRAJECTORY_BUILDER_3D.submaps.low_resolution
    TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range
    TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range

.. note::

    Cartographer ROS 提供了 RViz 插件来可视化子地图。你可以从他们的编号中选择你想看到的地图。在 3D 中,RViz 只显示 3D 混合概率网格的 2D 投影(在灰度上)。选项在 RViz 的左侧面板中可用,用于在低分辨率和低分辨率混合网格之间切换可视化。

**TODO**: *Documenting TSDF configuration*

全局 SLAM
-----------

当局部 SLAM 生成其子地图序列时,全局优化(通常称为"*优化问题*"或"*稀疏位姿调整*")任务在后台运行。
它的作用是重新排列子地图,使它们形成一个连贯的全局地图。
例如,这个优化负责修改当前构建的轨迹,以便根据回环检测正确对齐子地图。

一旦插入了一定数量的轨迹节点,优化就会批量运行。根据您需要运行它的频率,您可以调整这些批次的大小。

.. code-block:: lua

    POSE_GRAPH.optimize_every_n_nodes

.. note::

    将 POSE_GRAPH.optimize_every_n_nodes 设置为 0 是禁用全局 SLAM 并专注于局部 SLAM 行为的简便方法。这通常是调优 Cartographer 时要做的第一件事。

全局 SLAM 是一种"*GraphSLAM*",它本质上是一个位姿图优化,通过在**节点**和子地图之间建立**约束**,然后优化结果约束图来工作。
约束可以直观地理解为将所有节点绑在一起的小绳子。
稀疏位姿调整将这些绳子全部拉紧。
最终的网络被称为"*位姿图*"。

.. note::

    约束可以在 RViz 中可视化,这对调优全局 SLAM 非常有用。还可以切换 ``POSE_GRAPH.constraint_builder.log_matches`` 以获得约束构建器的定期报告,格式化为直方图。

- 非全局约束(也称为子地图内约束)在轨迹上紧密相连的节点之间自动建立。
  直观地说,这些"*非全局绳子*"保持轨迹的局部结构连贯。
- 全局约束(也称为回环检测约束或子地图间约束)在新子地图和被认为在空间上"*足够近*"(属于某个**搜索窗口**)且匹配度强(在运行扫描匹配时匹配良好)的先前节点之间定期搜索。
  直观地说,这些"*全局绳子*"在结构中引入结,并将两个分支牢固地拉近。

.. code-block:: lua

    POSE_GRAPH.constraint_builder.max_constraint_distance
    POSE_GRAPH.fast_correlative_scan_matcher.linear_search_window
    POSE_GRAPH.fast_correlative_scan_matcher_3d.linear_xy_search_window
    POSE_GRAPH.fast_correlative_scan_matcher_3d.linear_z_search_window
    POSE_GRAPH.fast_correlative_scan_matcher*.angular_search_window

.. note::

    在实践中,全局约束可以做比在单个轨迹上找到回环检测更多的东西。它们还可以对齐由多个机器人记录的不同轨迹,但我们将在本文档的范围之外保留这种用法和与 "全局定位" 相关的参数。

为了限制约束的数量(和计算)量,Cartographer 只考虑所有接近节点的子集进行约束构建。
这由采样比常数控制。
采样太少的节点可能会导致约束丢失和无效的回环检测。
采样太多的节点会降低全局 SLAM 的速度,并防止实时回环检测。

.. code-block:: lua

    POSE_GRAPH.constraint_builder.sampling_ratio

当节点和子地图被考虑用于约束构建时,它们通过第一个扫描匹配器 ``FastCorrelativeScanMatcher``。
这个扫描匹配器是为 Cartographer 设计的,并使实时回环检测扫描匹配成为可能。
``FastCorrelativeScanMatcher`` 依赖于 "*分支定界*" 机制,在不同的网格分辨率下工作,并有效地消除不正确的匹配。
这个机制在本文档前面介绍的 Cartographer 论文中进行了广泛介绍。
它在一个探索树中工作,深度可以控制。

.. code-block:: lua

    POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth
    POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.branch_and_bound_depth
    POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.full_resolution_depth

一旦 ``FastCorrelativeScanMatcher`` 有一个足够好的提议(超过匹配的最小分数),它就会被输入到 Ceres 扫描匹配器中进行细化。

.. code-block:: lua

    POSE_GRAPH.constraint_builder.min_score
    POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d
    POSE_GRAPH.constraint_builder.ceres_scan_matcher

当 Cartographer 运行 *优化问题* 时,Ceres 用于重新排列子地图,根据多个 *残差*。
残差使用加权成本函数计算。
全局优化具有成本函数来考虑大量数据源:全局(回环检测)约束、非全局(匹配器)约束、IMU 加速度和旋转测量、局部 SLAM 粗略位姿估计、里程计源或固定帧(如 GPS 系统)。
权重和 Ceres 选项可以配置为描述的 :ref:`local-slam` 部分。

.. code-block:: lua

    POSE_GRAPH.constraint_builder.loop_closure_translation_weight
    POSE_GRAPH.constraint_builder.loop_closure_rotation_weight
    POSE_GRAPH.matcher_translation_weight
    POSE_GRAPH.matcher_rotation_weight
    POSE_GRAPH.optimization_problem.*_weight
    POSE_GRAPH.optimization_problem.ceres_solver_options

.. note::

    你可以通过切换 ``POSE_GRAPH.log_residual_histograms`` 找到优化问题中使用的残差的有用信息。

作为 IMU 残差的一部分,优化问题为 IMU 位姿提供了一些灵活性,默认情况下,Ceres 可以自由优化 IMU 和跟踪帧之间的外参。
如果你不信任你的 IMU 位姿,你可以将结果记录下来,并用于改进你的外参。
如果 Ceres 没有正确优化你的 IMU 位姿,你可以将这个位姿固定。

.. code-block:: lua

    POSE_GRAPH.optimization_problem.log_solver_summary
    POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d

在残差中,异常值的影响由 **Huber 损失** 函数处理,该函数配置了一定的 Huber 比例。
Huber 比例越大,异常值的 `影响`_ 越大。

.. _the higher is the impact: https://github.com/ceres-solver/ceres-solver/blob/0d3a84fce553c9f7aab331f0895fa7b1856ef5ee/include/ceres/loss_function.h#L172

.. code-block:: lua

    POSE_GRAPH.optimization_problem.huber_scale

一旦轨迹完成,Cartographer 运行一个新的全局优化,通常比以前的优化有更多的迭代次数。
这是为了打磨 Cartographer 的最终结果,通常不需要实时,因此大量迭代通常是一个正确的选择。

.. code-block:: lua

    POSE_GRAPH.max_num_final_iterations
