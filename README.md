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