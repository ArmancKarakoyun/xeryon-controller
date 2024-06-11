from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xeryon_controller',
            executable='data_node',
            name='data_node',
            output='screen'
        ),
        Node(
            package='xeryon_controller',
            executable='linear_node',
            name='linear_node',
            output='screen'
        ),
        Node(
            package='xeryon_controller',
            executable='rotary_node',
            name='rotary_node',
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
