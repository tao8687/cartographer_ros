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

================
Getting involved
================

Cartographer is developed in the open and allows anyone to contribute to the project.
There are multiple ways to get involved!

If you have question or think you've found an issue in Cartographer, you are welcome to open a `GitHub issue`_.

.. _GitHub issue: https://github.com/cartographer-project/cartographer/issues

If you have an idea of a significant change that should be documented and discussed before finding its way into Cartographer, you should submit it as a pull request to `the RFCs repository`_ first.
Simpler changes can also be discussed in GitHub issues so that developers can help you get things right from the first try.

.. _the RFCs repository: https://github.com/cartographer-project/rfcs

If you want to contribute code or documentation, this is done through `GitHub pull requests`_.
Pull requests need to follow the `contribution guidelines`_.

.. _GitHub pull requests: https://github.com/cartographer-project/cartographer/pulls
.. _contribution guidelines: https://github.com/cartographer-project/cartographer/blob/master/CONTRIBUTING.md

=================
参与贡献
=================

我们欢迎贡献代码! 以下是一些指导原则。

代码审查
============

所有的代码都需要经过审查。我们使用 GitHub pull requests 进行代码审查。
请查看 `GitHub Help`_ 了解如何创建和管理分支。

.. _GitHub Help: https://help.github.com/articles/about-pull-requests/

代码风格
===========

C++ 代码
---------

我们遵循 `Google C++ Style Guide`_。
为了确保您的代码符合风格指南,请运行:

.. code-block:: bash

    scripts/check_code_style.sh

.. _Google C++ Style Guide: https://google.github.io/styleguide/cppguide.html

Lua 代码
---------

对于 Lua 代码,我们遵循 `lua-users style guide`_。

.. _lua-users style guide: http://lua-users.org/wiki/LuaStyleGuide

提交消息
===========

我们遵循 `Chris Beams' 指南`_。
简而言之:

1. 用一行空行分隔主题和正文
2. 限制主题行为 50 个字符
3. 主题行首字母大写
4. 不要在主题行末尾加句号
5. 使用祈使语气
6. 正文每行限制为 72 个字符
7. 使用正文解释是什么和为什么,而不是如何做

.. _Chris Beams' 指南: http://chris.beams.io/posts/git-commit/

单元测试
===========

我们使用 Google Test 进行单元测试。每个新功能都应该有相应的单元测试。
要运行测试,请执行:

.. code-block:: bash

    catkin_make_isolated --make-args run_tests
