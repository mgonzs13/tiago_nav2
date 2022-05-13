# tiago_nav2

## Installation

```shell
$ cd ~/ros2_ws/src
$ git clone git@github.com:mgonzs13/tiago_nav2.git
$ git clone --recurse-submodules git@github.com:mgonzs13/ros2_tiago.git
$ cd ~/ros2_ws
$ rosdep install --from-paths src -r -y
$ colcon build
```

## Usage

```shell
$ ros2 launch tiago_nav2 nav2.launch.py
```

## Gazebo

```shell
$ ros2 launch tiago_granny granny.launch.py
```
