# urdf_ws

**ロボットモデリング講習会：URDFの作成方法**<br>
[マニピュレータ型ロボットのURDF作成](https://gbiggs.github.io/rosjp_urdf_tutorial_text/manipulator_urdf.html)

**ROS Wiki**<br>
[urdf/XML](http://wiki.ros.org/urdf/XML)

## setup
```
$ git clone https://github.com/yoshi-ono/urdf_ws.git
$ cd ~/urdf_ws/src
$ catkin_create_pkg --rosdistro noetic manipulator_description
$ cd ~/urdf_ws
$ catkin_make
```

## rviz起動
```
$ cd ~/urdf_ws
$ source devel/setup.bash
$ roslaunch urdf_tutorial display.launch model:=src/manipulator_description/urdf/manipulator.urdf.xacro gui:=true
```

## noetic - ubunts 20.04
この環境で作成

## melodic - ubunts 18.04
下記(kinetic)と同様

## kinetic - ubunts 16.04
packageが不足
```
[ERROR] [1612657018.908127]: Could not find the GUI, install the 'joint_state_publisher_gui' package

$ sudo apt update
$ sudo apt install ros-kinetic-joint-state-publisher-gui
```

xacroファイルに日本語コメント``<!-- 日本語コメント -->``使用不可
```
Traceback (most recent call last):
  File "/opt/ros/kinetic/share/xacro/xacro.py", line 61, in <module>
    xacro.main()
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/xacro/__init__.py", line 1073, in main
    out.write(doc.toprettyxml(indent='  '))
UnicodeEncodeError: 'ascii' codec can't encode characters in position 675-680: ordinal not in range(128)
```

