# Nav2 Core

作用: 这个文件夹包含导航框架的核心接口和抽象类。
内容: 包括导航任务、行为树节点、插件接口等的抽象定义。

This package hosts the abstract interface (virtual base classes) for plugins to be used with the following:
- global planner (e.g., `nav2_navfn_planner`)
- controller (e.g., path execution controller, e.g `nav2_dwb_controller`)
- smoother (e.g., `nav2_ceres_costaware_smoother`)
- goal checker (e.g. `simple_goal_checker`)
- behaviors (e.g. `drive_on_heading`)
- progress checker (e.g. `simple_progress_checker`)
- waypoint task executor (e.g. `take_pictures`)
- exceptions in planning and control

The purposes of these plugin interfaces are to create a separation of concern from the system software engineers and the researcher / algorithm designers. Each plugin type is hosted in a "task server" (e.g. planner, recovery, control servers) which handles requests and multiple algorithm plugin instances. The plugins are used to compute a value back to the server without having to worry about ROS 2 actions, topics, or other software utilities. A plugin designer can simply use the tools provided in the API to do their work, or create new ones if they like internally to gain additional information or capabilities.
