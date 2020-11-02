编译后
1 先运行  roslaunch ruban_gnss_sim gazebo_sim.launch  启动仿真环境加载小车与世界文件
2 运行 roslaunch ruban_gnss_sim record_path_gazebo.launch  录制轨迹
3 waypoints文件复制改名方式和真车上一致
4 roslaunch ruban_gnss_sim load_update_pose.launch 加载录制过的路线和小车实时位置
5 roslaunch ruban_gnss_sim persuit.launch 开始循迹
