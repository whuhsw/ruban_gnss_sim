编译后

1  roslaunch ruban_gnss_sim gazebo_sim.launch  启动仿真环境加载小车与世界文件

2 运行 roslaunch ruban_gnss_sim record_path_gazebo.launch  录制轨迹
  保存地图文件  在功能包文件夹中的path_repository文件夹中  waypoints_时间.csv  复制到path文件夹，改名字为 waypoints.csv

3 roslaunch ruban_gnss_sim load_update_pose.launch 加载录制过的路线和小车实时位置

4 roslaunch ruban_gnss_sim persuit.launch 开始循迹

所有话题与可视化都是自utm frame
