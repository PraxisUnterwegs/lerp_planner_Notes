# 将自定义规划器插件集成到 MoveIt 2 的指南 - Cairui Gao

## 1. 添加新规划器

1. 在目录 `~/ws_moveit2/src/moveit2/moveit_planners` 下创建一个名为 `newplanner/newplanner_interface` 的新文件夹，并在其中创建两个子文件夹：`src` 和 `include`。

2. 在 `src` 文件夹中放置文件 `newplanner_interface.cpp`、`newplanner_planning_context.cpp`、`newplanner_planner_manager.cpp` 等。

3. 在 `include` 文件夹中放置文件 `newplanner_interface.h`、`newplanner_planning_context.h` 等。

4. 在目录 `~/ws_moveit2/src/moveit2/moveit_planners/newplanner/newplanner_interface` 中添加文件 `CMakeLists.txt`、`package.xml` 和 `newplanner_interface_plugin_description.xml`。
   **注意**：确保 `newplanner_interface_plugin_description.xml` 和 `CMakeLists.txt` 中的库名称一致。

   

## 2.在 MoveIt 2 中配置新规划器

在目录 `~/ws_moveit2/src/moveit2/moveit_configs_utils/default_configs` 中添加文件 `newplanner_planning.yaml`。