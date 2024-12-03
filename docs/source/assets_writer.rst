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

.. cartographer SHA: 30f7de1a325d6604c780f2f74d9a345ec369d12d
.. cartographer_ros SHA: 44459e18102305745c56f92549b87d8e91f434fe

.. _assets_writer:

利用 Cartographer ROS 生成的地图
================================================

随着传感器数据的输入,像 Cartographer 这样的 SLAM 算法的状态会不断演变,以保持对机器人轨迹和周围环境的*当前最佳估计*。
因此,Cartographer 能提供的最准确的定位和建图是在算法完成时获得的。
Cartographer 可以将其内部状态序列化为 ``.pbstream`` 文件格式,这本质上是一个压缩的 protobuf 文件,包含了 Cartographer 内部使用的数据结构的快照。

为了在实时运行时保持高效,Cartographer 会立即丢弃大部分传感器数据,只处理其输入的一小部分,因此内部使用(并保存在 ``.pbstream`` 文件中)的地图非常粗糙。
然而,当算法完成并确定最佳轨迹后,它可以在*事后*与完整的传感器数据重新组合,创建高分辨率地图。

Cartographer 通过 ``cartographer_assets_writer`` 实现这种重组。
资产写入器需要以下输入:

1. 用于执行 SLAM 的原始传感器数据(ROS ``.bag`` 文件)
2. 在对这些传感器数据执行 SLAM 时捕获的 cartographer 状态(保存在 ``.pbstream`` 文件中)
3. 传感器外参(即来自 bag 的 TF 数据或 URDF 描述)
4. 以及在 ``.lua`` 文件中定义的管道配置

资产写入器使用 ``.pbstream`` 中找到的轨迹批量处理 ``.bag`` 数据。
管道可用于对 SLAM 点云数据进行着色、过滤并导出为各种格式。
在管道中可以交错使用多个这样的点处理步骤 - `cartographer/io`_ 中已经提供了几个。

Sample Usage
------------

当使用离线节点运行 Cartographer 时,会自动保存一个 ``.pbstream`` 文件。
例如,使用 3D 背包示例:

.. code-block:: bash

   wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/b3-2016-04-05-14-14-00.bag
   roslaunch cartographer_ros offline_backpack_3d.launch bag_filenames:=${HOME}/Downloads/b3-2016-04-05-14-14-00.bag

观察命令行输出直到节点终止。
它将写入 ``b3-2016-04-05-14-14-00.bag.pbstream``,这代表了 Cartographer 处理完所有数据并完成所有优化后的状态。

当作为在线节点运行时,Cartographer 不知道您的 bag(或传感器输入)何时结束,因此您需要使用暴露的服务来显式完成当前轨迹并让 Cartographer 序列化其当前状态:

.. code-block:: bash

   # 完成第一条轨迹。不再接受新数据。
   rosservice call /finish_trajectory 0

   # 要求 Cartographer 序列化其当前状态。
   # (按 tab 键快速展开参数语法)
   rosservice call /write_state "{filename: '${HOME}/Downloads/b3-2016-04-05-14-14-00.bag.pbstream', include_unfinished_submaps: "true"}"

一旦获得了 ``.pbstream`` 文件,您就可以使用 3D 背包的`示例管道`_运行资产写入器:

.. _示例管道: https://github.com/cartographer-project/cartographer_ros/blob/44459e18102305745c56f92549b87d8e91f434fe/cartographer_ros/configuration_files/assets_writer_backpack_3d.lua

.. code-block:: bash

   roslaunch cartographer_ros assets_writer_backpack_3d.launch \
      bag_filenames:=${HOME}/Downloads/b3-2016-04-05-14-14-00.bag \
      pose_graph_filename:=${HOME}/Downloads/b3-2016-04-05-14-14-00.bag.pbstream

所有输出文件都以 ``--output_file_prefix`` 为前缀,默认为第一个 bag 的文件名。
对于最后一个示例,如果您在管道配置文件中指定 ``points.ply``,这将转换为 ``${HOME}/Downloads/b3-2016-04-05-14-14-00.bag_points.ply``。

Configuration
-------------

资产写入器被建模为 `PointsProcessor`_ 步骤的管道。
`PointsBatch`_ 数据流经每个处理器,它们都有机会在传递之前修改 ``PointsBatch``。

例如,3D 背包的 `assets_writer_backpack_3d.lua`_ 管道使用 ``min_max_range_filter`` 来移除距离传感器太近或太远的点。
之后,它保存"*X射线*"(地图的半透明侧视图),然后根据传感器帧 ID 重新着色 ``PointsBatch``\s,并使用这些新颜色写入另一组 X射线。

可用的 ``PointsProcessor``\s 都定义在 `cartographer/io`_ 子目录中,并在它们各自的头文件中有文档说明:

* **color_points**: 根据 frame_id 用固定颜色为点着色。
* **dump_num_points**: 传递点,但跟踪它看到了多少点并在 Flush 时输出。
* **fixed_ratio_sampler**: 只让固定的'sampling_ratio'点通过。'sampling_ratio'为1时此过滤器无效。
* **frame_id_filter**: 过滤所有黑名单中的 frame_id 或非白名单中的 frame_id。注意您可以指定白名单或黑名单,但不能同时指定两者。
* **write_hybrid_grid**: 创建点的混合网格,体素大小为'voxel_size'。使用'range_data_inserter'选项配置通过混合网格的测距数据光线追踪。
* **intensity_to_color**: 对来自'frame_id'传感器的每个点应用('intensity' - min) / (max - min) * 255,并用此值将点着色为灰色。如果'frame_id'为空,则应用于所有点。
* **min_max_range_filtering**: 过滤所有距离'origin'超过'max_range'或近于'min_range'的点。
* **voxel_filter_and_remove_moving_objects**: 对数据进行体素过滤,只传递我们认为在非移动物体上的点。
* **write_pcd**: 将 PCD 文件流式写入磁盘。头部在'Flush'时写入。
* **write_ply**: 将 PLY 文件流式写入磁盘。头部在'Flush'时写入。
* **write_probability_grid**: 创建指定'resolution'的概率网格。由于所有点都投影到x-y平面,z分量被忽略。使用'range_data_inserter'选项配置通过概率网格的测距数据光线追踪。
* **write_xray_image**: 创建点的X射线切片,像素大小为'voxel_size'。
* **write_xyz**: 写入 ASCII xyz 点。

点云的第一人称可视化
------------------------------------------

两个 ``PointsProcessor``\s 特别有趣:``pcd_writing`` 和 ``ply_writing`` 可以将点云保存为 ``.pcd`` 或 ``.ply`` 文件格式。
这些文件格式随后可以被专门的软件如 `point_cloud_viewer`_ 或 `meshlab`_ 用来浏览高分辨率地图。

用于此结果的典型资产写入器管道由 IntensityToColorPointsProcessor_ 给点非白色和 PlyWritingPointsProcessor_ 将结果导出到 ``.ply`` 点云组成。
这样的管道示例在 `assets_writer_backpack_2d.lua`_ 中。

一旦您获得了 ``.ply`` 文件,请按照 `point_cloud_viewer`_ 的 README 生成一个磁盘上的体素数据结构,该结构可以由同一仓库中的查看器之一(基于 SDL 或网页)查看。
注意,`point_cloud_viewer`_ 需要颜色才能正常工作。

.. image:: point_cloud_viewer_demo_3d.jpg
