import os
from os import environ, pathsep

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition


def get_model_paths(packages_names):
    model_paths = ""
    for package_name in packages_names:
        if model_paths != "":
            model_paths += pathsep

        package_path = get_package_prefix(package_name)
        model_path = os.path.join(package_path, "share")

        model_paths += model_path

    return model_paths


def generate_launch_description():

    launch_file_dir = os.path.join(
        get_package_share_directory("tiago_granny"), "launch")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    ### ARGS ###
    world = LaunchConfiguration("world")
    world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(
            get_package_share_directory("tiago_granny"), "worlds", "empty.world"),
        description="Gazebo world")

    launch_gui = LaunchConfiguration("launch_gui")
    launch_gui_cmd = DeclareLaunchArgument(
        "launch_gui",
        default_value="True",
        description="Whether launch gzclient")

    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_rviz_cmd = DeclareLaunchArgument(
        "launch_rviz",
        default_value="True",
        description="Whether launch rviz2")

    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_x_cmd = DeclareLaunchArgument(
        "initial_pose_x",
        default_value="1.336",
        description="Initial pose x")

    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_y_cmd = DeclareLaunchArgument(
        "initial_pose_y",
        default_value="6.544",
        description="Initial pose y")

    initial_pose_z = LaunchConfiguration("initial_pose_z")
    initial_pose_z_cmd = DeclareLaunchArgument(
        "initial_pose_z",
        default_value="0.0",
        description="Initial pose z")

    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    initial_pose_yaw_cmd = DeclareLaunchArgument(
        "initial_pose_yaw",
        default_value="0.0",
        description="Initial pose yaw")

    ### NODES ###
    rviz_cmd = Node(
        name="rviz",
        package="rviz2",
        executable="rviz2",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(PythonExpression([launch_rviz]))
    )

    ### LAUNCHS ###
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world, "gui": launch_gui}.items()
    )

    spawn_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "spawn.launch.py")
        ),
        launch_arguments={
            "initial_pose_x": initial_pose_x,
            "initial_pose_y": initial_pose_y,
            "initial_pose_z": initial_pose_z,
            "initial_pose_yaw": initial_pose_yaw
        }.items()
    )

    ### GAZEBO MODELS ###
    packages = ["tiago_description", "pmb2_description",
                "hey5_description", "pal_gripper_description"]
    model_path = get_model_paths(packages)

    if "GAZEBO_MODEL_PATH" in environ:
        model_path += pathsep + environ["GAZEBO_MODEL_PATH"]

    gazebo_model_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    ### LD ###
    ld = LaunchDescription()
    ld.add_action(gazebo_model_var)

    ld.add_action(launch_gui_cmd)
    ld.add_action(launch_rviz_cmd)
    ld.add_action(world_cmd)
    ld.add_action(initial_pose_x_cmd)
    ld.add_action(initial_pose_y_cmd)
    ld.add_action(initial_pose_z_cmd)
    ld.add_action(initial_pose_yaw_cmd)

    ld.add_action(gazebo_cmd)
    ld.add_action(spawn_cmd)
    ld.add_action(rviz_cmd)

    return ld
