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

==========================
常见问题
==========================

为什么3D数据包中的激光数据速率高于VLP-16报告的20 Hz最大旋转速度?
----------------------------------------------------------------------------------------------------------

示例数据包中的VLP-16配置为20 Hz旋转。然而,VLP-16发送UDP数据包的频率要高得多,且与旋转频率无关。
示例数据包包含每个UDP数据包对应的一个 `sensor_msgs/PointCloud2`__,而不是每转一圈对应一个。

__ http://www.ros.org/doc/api/sensor_msgs/html/msg/PointCloud2.html

在`相应的Cartographer配置文件`__中,您可以看到
`TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 160`,这意味着我们
将160个每UDP数据包的点云累积成一个更大的点云,该点云通过结合恒速和IMU
测量来进行运动估计以用于匹配。由于有两个VLP-16,160个UDP数据包大约
足够两圈,每个VLP-16一圈。

__ https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros/configuration_files/backpack_3d.lua

为什么3D SLAM需要IMU数据而2D不需要?
----------------------------------------------------

在2D中,Cartographer支持运行相关性扫描匹配器,它通常用于寻找回环闭合约束,也可用于局部SLAM。
它在计算上很昂贵,但通常可以使里程计或IMU数据的整合变得不必要。
2D还有假设世界是平坦的优势,即向上方向是隐式定义的。

在3D中,主要需要IMU来测量重力。
重力是一个很有吸引力的测量量,因为它不会漂移,是一个非常强的信号,通常包含了大部分测得的加速度。
需要重力有两个原因:

1. 在3D中对世界没有任何假设。
为了正确地对齐最终的轨迹和地图,使用重力来定义z方向。

2. 一旦确定了重力方向,就可以从IMU读数中很好地推导出横滚和俯仰角。
这通过减少这些维度的搜索窗口来减轻扫描匹配器的工作。

如何在没有rviz支持的情况下构建cartographer_ros?
-----------------------------------------------------

最简单的解决方案是在 `cartographer_rviz` 包目录中创建一个名为 `CATKIN_IGNORE`__ 的空文件。

__ http://wiki.ros.org/catkin/workspaces

如何修复"You called InitGoogleLogging() twice!"错误?
---------------------------------------------------------------

使用 `glog` 后端构建 `rosconsole` 可能导致此错误。
使用 `log4cxx` 或 `print` 后端(可通过 `ROSCONSOLE_BACKEND` CMake 参数选择)来避免此问题。
