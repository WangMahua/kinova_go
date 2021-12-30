# kinova_go
## how to use
download following repo
kinova core:https://github.com/Kinovarobotics/kinova-ros
kinova gui:https://github.com/EasonHuang-tw/kinova_gui

## 
1. open kinova api and gui
cd catkin_ws
source devel/setup.bash
roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2n6s300
cd ~/kinova_gui
python test.py

2. open coordinate transformation code
cd catkin_ws
source devel/setup.bash
cd src/kinova_go/scripts
python kinova_frame.py

3. open kinova control
cd catkin_ws
source devel/setup.bash
cd src/kinova_go/scripts
python camera_demo.py
