# 将自定义规划器插件集成到 MoveIt 2 的指南

## 仓库说明

这个仓库用于修复moveit2、moveit-humble官方教程里版本滞后的问题（即create planner plugin部分的教程使用的代码案例仍然是moveit1-noetic版本的api，导致无法编译成功。这部分代码你可以在moveit_tutorials的官方仓库中查看）

现已完成了api的改进，使得这个仓库得以适应ros2，moveit2的api，这对新手的学习无疑是十分有利的，可以提供一个完整的，可成功编译的案例以供学习参考。

---

## 1. 添加新规划器

1. 在目录 `~/ws_moveit2/src/moveit2/moveit_planners` 下创建一个名为 `newplanner/newplanner_interface` 的新文件夹，并在其中创建两个子文件夹：`src` 和 `include`。

2. 在 `src` 文件夹中放置文件 `newplanner_interface.cpp`、`newplanner_planning_context.cpp`、`newplanner_planner_manager.cpp` 等。

3. 在 `include` 文件夹中放置文件 `newplanner_interface.h`、`newplanner_planning_context.h` 等。

4. 在目录 `~/ws_moveit2/src/moveit2/moveit_planners/newplanner/newplanner_interface` 中添加文件 `CMakeLists.txt`、`package.xml` 和 `newplanner_interface_plugin_description.xml`。
   **注意**：确保 `newplanner_interface_plugin_description.xml` 和 `CMakeLists.txt` 中的库名称一致。

   

## 2.在 MoveIt 2 中配置新规划器

在目录 `~/ws_moveit2/src/moveit2/moveit_configs_utils/default_configs` 中添加文件 `newplanner_planning.yaml`。

这一步是**必要的**，因为当你使用 MoveIt Assistant setup 工具创建功能包时，它默认会加载 `moveit_configs_utils/default_configs/` 目录中的 YAML 配置文件，其中包含 MoveIt 的默认规划参数（例如 `ompl_planning.yaml` 等）。这些参数会被自动应用到 MoveIt 运行时的配置中，除非你在启动文件（launch 文件）中明确指定了其他 YAML 文件来覆盖默认配置。

---

## 3.在Rviz里面让demo跑起来，生成轨迹路线（让机械臂动起来）

利用moveit assistant setup工具自己构建一个`newplanner_moveit_config`功能包，然后输入`ros2 launch newplanner_moveit_config demo.launch.py`，就能打开 `Rviz`界面，然后在motion planning bar里的`contex`

选项卡里选择自定义的插件就行了。不出意外的话，应该有`ompl`、`chomp`、`lerp`这些选项了，然后就你切换为自定义的规划器，就可以跑demo了。

<span style="color:orange">简而言之，你其实无需自行撰写一个`newplanner_example.cpp`来作为`demo`执行的源文件。请直接依赖自动配置工具moveit assistant setup生成的功能包就行了，这个功能包里一定会有`demo.launch.py`文件。你启动这个启动文件，然后在`Rviz`可视化界面里面更换规划器插件，然后自己进行轨迹demo就行了。</span> 

***什么 motion planning / motion planning pipeline 的 api flow 你其实完全不用去管，除非你要手动设置精确的约束，比如说目标约束、轨迹约束、公差等等信息**。* 

至于添加物体障碍物等到场景中去，你可以通过`Rviz`，也可以通过写源代码的方式。而且，源代码和moveit config功能包也不是完全独立的（即各自独立工作，只需要一者）。实际上，两者是可以协同工作的。自己写的源代码可以放在自定义功能包里并撰写新的启动文件，然后再 include  `moveit config` 功能包里的 `demo.launch.py`，这样可以大大简化自己的启动文件梳理编写工作。

只不过，如果只需要使用 demo 来测试自定义规划器插件的成功工作与否，只需要借用 moveit assistant setup 配置 moveit config功能包就可以了，省的麻烦。