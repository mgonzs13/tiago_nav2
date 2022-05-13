from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_pal.include_utils import include_launch_py_description


def generate_launch_description():

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

    robot_state_publisher_cmd = include_launch_py_description(
        "tiago_granny",
        ["launch", "robot_state_publisher.launch.py"])

    tiago_bringup_cmd = include_launch_py_description(
        "tiago_bringup", ["launch", "tiago_bringup.launch.py"])

    spawn_entity_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "tiago",
                   "-topic", "robot_description",
                   "-timeout", "120",
                   "-x", initial_pose_x,
                   "-y", initial_pose_y,
                   "-z", initial_pose_z,
                   "-Y", initial_pose_yaw],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    ld = LaunchDescription()

    ld.add_action(initial_pose_x_cmd)
    ld.add_action(initial_pose_y_cmd)
    ld.add_action(initial_pose_z_cmd)
    ld.add_action(initial_pose_yaw_cmd)

    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(tiago_bringup_cmd)
    ld.add_action(spawn_entity_cmd)

    return ld
