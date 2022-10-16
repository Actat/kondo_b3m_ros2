from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    kondo_b3m_ros2_node = Node(
        package='kondo_b3m_ros2',
        executable='kondo_b3m',
        remappings=[('b3m_joint_state', 'joint_states')],
        parameters=[{
            'motor_list': ['{"id": 0, "name": "joint0", "direction": true, "offset": 0}',
                           '{"id": 1, "name": "joint1"}',
                           '{"id": 2, "direction": false}',
                           '{"id": 3, "offset": 0.5}',
                           '{"id": 4}'],
        }]
    )

    ld = LaunchDescription()
    ld.add_action(kondo_b3m_ros2_node)

    return ld
