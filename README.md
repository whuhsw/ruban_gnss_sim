# ruban_gnss_sim
1  地图录制 ：

新终端命令行 #  roslaunch cartographer_ros ruban_online.launch

2  地图保存：


新终端命令行 # rosservice call /finish_trajectory 0
待执行完毕后继续
命令行 # rosservice call /write_state "{filename: '${HOME}/Maps/mymap1.pbstream'}"
待执行完毕后继续
命令行 # rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${HOME}/Maps/mymap1 -pbstream_filename=${HOME}/Maps/mymap1.pbstream -resolution=0.05

3  导航：

在工控机中找到~/catkin_app_ws/src/ruban_navigation/launch文件夹

打开ruban_navigation.launch文件，修改

<arg name="map_file" default="/home/bingo/catkin_app_ws/src/ruban_navigation/maps/1411.yaml"/>
  <arg name="map_paint_file" default="/home/bingo/catkin_app_ws/src/ruban_navigation/maps/1411_paint.yaml"/> 

修改地图文件路径为刚录制的地图文件路径
然后新终端命令行： roslaunch ruban_navigation ruban_navigation.launch

