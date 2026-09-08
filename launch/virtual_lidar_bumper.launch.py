from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    parameters_file = PathJoinSubstitution(
        [FindPackageShare("virtual_lidar_bumper"), "config", "parameters.yaml"]
    )

    virtual_lidar_bumper_node = Node(
        package="virtual_lidar_bumper",
        executable="virtual_lidar_bumper_node",
        parameters=[
            parameters_file,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time")
            }
        ],
        remappings=[
            ("input_pointcloud", LaunchConfiguration("pointcloud_topic")),
            ("bumper_triggered", LaunchConfiguration("output_topic")),
        ],
        output="screen",
    )

    return [virtual_lidar_bumper_node]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "pointcloud_topic",
            default_value="/odin1/cloud_radius_filtered",
            description="Topic name for the input pointcloud",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "output_topic",
            default_value="/virtual_bumper",
            description="Topic name for the published bumper trigger (std_msgs/Bool)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time (set true for Gazebo / bag playback)",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
