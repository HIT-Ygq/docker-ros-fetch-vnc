# docker-ros-fetch-vnc
fetch机器人巡检
镜像使用方法： docker run -it -p 5900:5900 -p 6080:80 yin211/docker-ros-fetch-novnc bash 配合密码使用： docker run -p 6080:80 -e VNC_PASSWORD=mypassword yin211/docker-ros-fetch-novnc bash VNC软件：输入 :5900 noVNC输入：http://localhost:6080/ 或者 : http://127.0.0.1:6080/

巡检机器人操作步骤：

启动Gazebo，加载fetch及环境模型，另开一终端，输入： roslaunch gazebo_my playground.launch
启动建图命令，另开一终端，输入： roslaunch gazebo_my build_map.launch
启动rviz，使建图过程可视化，另开一终端，输入： roslaunch gazebo_my rviz.launch 
找到Camera，选择打勾，点击展开 在Image Topic的下拉列表中选择 /head_camera/rgb/image_raw 调整Gazebo、rviz的角度与布局
启动键盘控制，另开一终端，输入： rosrun teleop_twist_keyboard teleop_twist_keyboard.py 在键盘控制终端上，按照按键说明驱动fetch运动，进行建图
保存地图，另开一终端，输入：（说明：mymap为地图名称，保存路径不建议修改，机器人使用导航功能时需要读取该路径下的地图文件） 
rosrun map_server map_saver -f ~/catkin_ws/src/gazebo_my/config/mymap
终止建图功能，启动导航功能，输入： roslaunch gazebo_my fetch_nav.launch
在rviz界面中设置fetch当前大致位置，并设定目标点，fetch将自主导航至该位置。 通过键盘将fetch移动至黄色框线内，并调整至箭头所示方向，观察仪表，输入： 
rosrun gazebo_my operate.py
再次设定目标点，控制fetch到达“步骤2”位置，通过提升躯干观察仪表，输入： 
rosrun gazebo_my operate.py rise
继续设定目标点，控制fetch到达“步骤3”位置，执行按下警报按钮的操作，输入： 
rosrun gazebo_my operate.py button 10.回到fetch起始点位置，结束所有任务，输入： 
rosrun gazebo_my finish.py 
在各个终端中输入：Ctrl+C，关闭终端
