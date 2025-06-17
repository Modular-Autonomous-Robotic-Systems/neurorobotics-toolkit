import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Generates the launch description for starting the multi-topic logger node.
    """
    # Get the package share directory for this package.
    share_dir = get_package_share_directory('sensors')

    # --- Declare Launch Arguments ---

    # Argument for the output bag file name (without extension).
    output_bag_name_arg = DeclareLaunchArgument(
        'output_bag_name',
        default_value='/ws/data/telemetry',
        description='Name of the output rosbag file.'
    )

    # Define the list of all supported topics.
    all_supported_topics_list = [
        "/ap/airspeed", "/ap/battery", "/ap/clock", "/ap/cmd_gps_pose", "/ap/cmd_vel",
        "/ap/geopose/filtered", "/ap/goal_lla", "/ap/gps_global_origin/filtered",
        "/ap/imu/experimental/data", "/ap/joy", "/ap/navsat", "/ap/pose/filtered",
        "/ap/status", "/ap/tf", "/ap/tf_static", "/ap/time", "/ap/twist/filtered"
    ]
    # Format the list into a string that looks like a Python list, e.g., "['/topic1', '/topic2']"
    # This is the format the ROS 2 parameter system expects for a vector of strings.
    default_topics_str = f"[{', '.join([f'{repr(topic)}' for topic in all_supported_topics_list])}]"

    topics_to_record_arg = DeclareLaunchArgument(
        'topics_to_record',
        default_value=default_topics_str,
        description="A string-formatted list of topics to record, e.g., \"['/topic1', '/topic2']\""
    )

    # --- Node Definition ---

    # Define the LifecycleNode for the logger.
    logger_node = Node(
        package='sensors',
        executable='managed_logger_node',
        name=None,
        namespace='',
        output='screen',
        # prefix = ['gdb -ex r -ex bt --args'],
        parameters=[{
            'output_bag_name': LaunchConfiguration('output_bag_name'),
            # Pass the launch argument directly. The C++ node's get_parameter call
            # for a std::vector<std::string> will correctly parse the string-formatted list.
            'topics_to_record': LaunchConfiguration('topics_to_record'),
        }]
    )

    ld = LaunchDescription()
    ld.add_action(output_bag_name_arg)
    ld.add_action(topics_to_record_arg)
    ld.add_action(logger_node)

    return ld

