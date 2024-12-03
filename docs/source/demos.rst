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

======================================
在演示数据包上运行 Cartographer ROS
======================================

现在 Cartographer 和 Cartographer 的 ROS 集成已经安装完成,您可以
下载示例数据包(例如`德意志博物馆 <https://en.wikipedia.org/wiki/Deutsches_Museum>`_的2D和3D背包采集数据)到
指定位置,在本例中是 ``~/Downloads``,然后使用 ``roslaunch`` 启动演示。

启动文件会自动启动 ``roscore`` 和 ``rviz``。

.. warning:: 当您想要运行 cartographer_ros 时,可能需要先运行 ``source install_isolated/setup.bash`` 来设置 ROS 环境(如果您使用的是 zsh shell,则将 bash 替换为 zsh)

德意志博物馆
================

下载并启动2D背包演示:

.. code-block:: bash

    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag
    roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=${HOME}/Downloads/cartographer_paper_deutsches_museum.bag

下载并启动3D背包演示:

.. code-block:: bash

    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/with_intensities/b3-2016-04-05-14-14-00.bag
    roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=${HOME}/Downloads/b3-2016-04-05-14-14-00.bag

纯定位模式
=================

纯定位模式使用2个不同的数据包。第一个用于生成地图,第二个用于运行纯定位。

从德意志博物馆下载2D数据包:

.. code-block:: bash

    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/b2-2016-04-05-14-44-52.bag
    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/b2-2016-04-27-12-31-41.bag

生成地图(等待 cartographer_offline_node 完成)然后运行纯定位:

.. code-block:: bash

    roslaunch cartographer_ros offline_backpack_2d.launch bag_filenames:=${HOME}/Downloads/b2-2016-04-05-14-44-52.bag
    roslaunch cartographer_ros demo_backpack_2d_localization.launch \
       load_state_filename:=${HOME}/Downloads/b2-2016-04-05-14-44-52.bag.pbstream \
       bag_filename:=${HOME}/Downloads/b2-2016-04-27-12-31-41.bag

从德意志博物馆下载3D数据包:

.. code-block:: bash

    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/b3-2016-04-05-13-54-42.bag
    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/b3-2016-04-05-15-52-20.bag

生成地图(等待 cartographer_offline_node 完成)然后运行纯定位:

.. code-block:: bash

    roslaunch cartographer_ros offline_backpack_3d.launch bag_filenames:=${HOME}/Downloads/b3-2016-04-05-13-54-42.bag
    roslaunch cartographer_ros demo_backpack_3d_localization.launch \
       load_state_filename:=${HOME}/Downloads/b3-2016-04-05-13-54-42.bag.pbstream \
       bag_filename:=${HOME}/Downloads/b3-2016-04-05-15-52-20.bag

静态地标
================

  .. raw:: html

      <iframe width="560" height="315" src="https://www.youtube.com/embed/E2-OD-ycivc" frameborder="0" allowfullscreen></iframe>

  .. code-block:: bash

    # 下载地标示例数据包
    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/mir/landmarks_demo_uncalibrated.bag

    # 启动地标演示
    roslaunch cartographer_mir offline_mir_100_rviz.launch bag_filename:=${HOME}/Downloads/landmarks_demo_uncalibrated.bag

Revo LDS
========

下载并启动一个从 Neato Robotics 扫地机器人上采集的低成本 Revo 激光测距传感器的示例数据包:

.. code-block:: bash

    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/revo_lds/cartographer_paper_revo_lds.bag
    roslaunch cartographer_ros demo_revo_lds.launch bag_filename:=${HOME}/Downloads/cartographer_paper_revo_lds.bag

PR2
===

下载并启动一个从 Willow Garage 的 PR2 研发人形机器人上采集的示例数据包:

.. code-block:: bash

    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/pr2/2011-09-15-08-32-46.bag
    roslaunch cartographer_ros demo_pr2.launch bag_filename:=${HOME}/Downloads/2011-09-15-08-32-46.bag

Taurob Tracker
==============

下载并启动一个从 Taurob Tracker 遥操作机器人上采集的示例数据包:

.. code-block:: bash

    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/taurob_tracker/taurob_tracker_simulation.bag
    roslaunch cartographer_ros demo_taurob_tracker.launch bag_filename:=${HOME}/Downloads/taurob_tracker_simulation.bag
