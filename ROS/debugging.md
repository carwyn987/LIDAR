I've had tons of issues along the way with this project, ranging from compatible OS's, Velodyne software, ROS versions, and bugs due to the system76 I'm running on (in the future, I want to try using a docker container). I've repartitioned my disk, installed various versions of Ubuntu, tested ROS1 (noetic, kinetic) and ROS2 (humble), broken my display (ubuntu-desktop), and more.

# EmPy - em namespace issues

One error I've encountered repeatedly is the "empy" - "em" error. I didn't end up finding the answer online, but eventually found the solution which was the following:

```
pip uninstall em
pip uninstall empy
pip uninstall colcon-core
pip uninstall colcon-common-extensions
pip install empy==3.3.4
pip install git+https://github.com/colcon/colcon-core.git@0.13.0
pip install colcon-common-extensions
```

Finally, the following cmd works
```
colcon build --symlink-install
carwyn@carwyn:~/development/lidar/ros2_ws/src$ ros2 pkg create --build-type ament_cmake my_package
```

NOTE: Doing the following did **NOT** work:

```
pip install -U git+https://github.com/colcon/colcon-core.git@cottsay/empy4
pip install colcon-core
```

and/or

```
pip uninstall em
pip uninstall empy
pip install empy==3.3.4
```