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

==========================
编译 Cartographer ROS
==========================

系统要求
===================

Cartographer ROS 的系统要求与 `Cartographer 的要求`_相同。

目前支持以下 `ROS 发行版`_:

* Melodic
* Noetic

.. _Cartographer 的要求: https://google-cartographer.readthedocs.io/en/latest/#system-requirements
.. _ROS 发行版: http://wiki.ros.org/Distributions

构建与安装
=======================

为了构建 Cartographer ROS,我们推荐使用 `wstool <http://wiki.ros.org/wstool>`_ 和 `rosdep
<http://wiki.ros.org/rosdep>`_。为了加快构建速度,我们还推荐使用 `Ninja <https://ninja-build.org>`_。

在安装了 ROS Noetic 的 Ubuntu Focal 上,使用以下命令安装上述工具:

.. code-block:: bash

    sudo apt-get update
    sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow

在较旧的发行版上:

.. code-block:: bash

    sudo apt-get update
    sudo apt-get install -y python-wstool python-rosdep ninja-build stow

工具安装完成后,在 'catkin_ws' 中创建一个新的 cartographer_ros 工作空间。

.. code-block:: bash

    mkdir catkin_ws
    cd catkin_ws
    wstool init src
    wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
    wstool update -t src

现在需要安装 cartographer_ros 的依赖项。
首先,我们使用 ``rosdep`` 安装所需的软件包。
如果您在安装 ROS 后已经执行过 'sudo rosdep init' 命令,它会显示一个错误。这个错误可以忽略。

.. code-block:: bash

    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

Cartographer 使用 `abseil-cpp`_ 库,需要使用此脚本手动安装:

.. code-block:: bash

    src/cartographer/scripts/install_abseil.sh 

由于版本冲突,您可能需要卸载 ROS abseil-cpp:

.. code-block:: bash

   sudo apt-get remove ros-${ROS_DISTRO}-abseil-cpp 

构建并安装。

.. code-block:: bash

    catkin_make_isolated --install --use-ninja

这将从 master 分支的最新 HEAD 构建 Cartographer。
如果您想要特定版本,需要在 cartographer_ros.rosinstall 中更改版本。

.. _abseil-cpp: https://abseil.io/
