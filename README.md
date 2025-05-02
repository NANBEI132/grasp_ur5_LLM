#####最下面是前作者的readme，respect。上面这部分是我的，我会简单介绍必要的启动方式，详细的阐释会在我的csdn文章中体现。
文件中的control.py;motion_ros2plan;以及gazebo_vlm_interfaceduo55502.py（大模型控制的部分）都是我所添加，借用了原作者搭建的场景与建模，在此表示感谢。之后我会发布自己的场景以及其他的功能，这都是后话了。。。
在创建了conda空间voice_qwen以及安装了相关的依赖之后，将gazebo_vlm_interfaceduo55502.py代码放进工作空间，当然里面的api的部分需要用自己的然后重新安装依赖，至此完成。
下载了仓库之后首先进入到最下面的前作者的readme部分，将他之前的依赖安装好，然后进入到catkin_ws
然后输入：再次安装依赖
rosdep install --from-paths src --ignore-src -r -y
；；；；catkin build 编译，如果报错，就自行解决依赖问题

假设你已经ok了，接下来开始使用；；
####1：打开gazebo：：
cd UR5-Pick-and-Place-Simulation/catkin_ws；；；；
source devel/setup.bash；；；；
roslaunch levelManager lego_world.launch；；；；

###2；添加积木
source devel/setup.bash；；；；
rosrun levelManager levelManager.py -l 1  ；；；；      （1-4代表不同数量和难度的积木）


####3 
source devel/setup.bash；；；；
rosrun motion_planning motion_plan2ros5.py  ；；；；           这个是启动接受话题的，让voice里面的坐标能过来，然后运动。


#######4：
创建一个voice_qwen的conda空间
conda create -n voice_qwen python=3.10；；；；
conda activate voice_qwen；；；；



######5:
启动
python gazebo_vlm_interfaceduo55502.py ；；；；
![image](https://github.com/user-attachments/assets/c5d5745c-a9df-4254-84f3-dc0f239d491f)
这里可以看见需要先点击一下图片
![image](https://github.com/user-attachments/assets/b48b0391-605c-4f73-9346-74d7e681d282)
也就是你需要在这个界面点击一下，然后回到那个终端开始按下回车，然后语音输入你的命令。使用就到这里；。详细信息请看我的csdn；
链接如下：


；；；；；；；；；；；；；；；；；；；；；；；；；；


###########################################################################################################################################










































<p align="center">
  <h2 align="center">UR5 Pick and Place Simulation in Ros/Gazebo</h2>

  
</p>
<br>

<img src="https://github.com/pietrolechthaler/UR5-Pick-and-Place-Simulation/blob/main/main.png">

## Table of contents
- [Description](#description)
- [Requirement](#requirements)
- [Folder](#folder)
- [Setup](#setup)
- [Usage](#usage)
- [Contributors](#contributors)

### Description
This repository demonstrates UR5 pick-and-place in ROS and Gazebo. The UR5 uses a Xbox Kinect cam to detect eleven types of Lego Bricks, and publish its position and angolation. 

The goals of this project are:
- simulate the iteration of a UR5 robot with Lego bricks
- The robotic arm must be able to move a block from position A to B and construct a castle by assembling different bricks

<img src="https://github.com/pietrolechthaler/UR5-Pick-and-Place-Simulation/blob/main/intro.gif">

### Folder
```
UR5-Pick-and-Place-Simulation/catkin_ws/
├── levelManager
├── vision
├── motion_planning
├── gazebo_ros_link_attacher
├── robot
```
- `levelManager:` the task of this package is to launch the world and spawn the different bricks
- `vision:` the task of this package is to recognize the object type and orientation of the bricks
- `motion_planning:` the task is to move the robot and pick and place the lego
- `gazebo_ros_link_attacher:` A gazebo plugin definable from URDF to inform a client of a collision with an object
- `robot:` the task is to define the robot model with appropriate PID settings


### Requirements

For running each sample code:
- `Ros Noetic:` http://wiki.ros.org/noetic/Installation
- `Gazebo:` https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros
- `Yolov5` https://github.com/ultralytics/yolov5
- `Catkin` https://catkin-tools.readthedocs.io/en/latest/

### Setup

After installing the libraries needed to run the project. Clone this repo:
```
git clone https://github.com/pietrolechthaler/UR5-Pick-and-Place-Simulation/
```

Setup the project:
```
cd UR5-Pick-and-Place-Simulation/catkin_ws
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
echo "source $PWD/devel/setup.bash" >> $HOME/.bashrc
```

Clone and install [YoloV5](https://github.com/ultralytics/yolov5):
```
cd ~
git clone https://github.com/ultralytics/yolov5
cd yolov5
pip3 install -r requirements.txt
```
### Usage

Launch the world
```
roslaunch levelManager lego_world.launch
```
Choose the level (from 1 to 4):
```
rosrun levelManager levelManager.py -l [level]
```
Start the kinematics process
```
rosrun motion_planning motion_planning.py
```
Start the localization process
```
rosrun vision vision.py -show
```
- `-show` : show the results of the recognition and localization process with an image

### Contributors

| Name                 | Github                               |
|----------------------|--------------------------------------|
| Davice Cerpelloni    | https://github.com/davidecerpelloni  |
| Leonardo Collizzolli | https://github.com/leocolliz         |
| Pietro Lechthaler    | https://github.com/pietrolechthaler  |
| Stefano Rizzi        | https://github.com/StefanoRizzi      |
