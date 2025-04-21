from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode

def generate_launch_description():
    ns = 'drone1'
    return LaunchDescription([
        # TelloControllerNode with remapped topics
        Node(
            package="controllers",
            executable="tello_controller_node",
            name="tello_controller",
            namespace=ns,
            output="screen",
        ),
        # Lifecycle node for Tello Joy
        LifecycleNode(
            package='tello_driver',
            executable='tello_joy_main',
            name='tello_joy',
            namespace=ns,
            output='screen',
        ),
        # Joy node with consistent remapping
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            namespace=ns,
            output='screen',

        ),
        
         Node(
             package='tello_driver',
             executable='tello_driver_main',
             name='tello_driver_main',
             namespace=ns,
             output='screen',
        ),
])
