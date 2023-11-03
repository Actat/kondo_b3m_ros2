from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    kondo_b3m_ros2_node = Node(
        package='kondo_b3m_ros2',
        executable='kondo_b3m',
        remappings=[('b3m_joint_state', 'joint_states')],
        parameters=[{
            'port_name': '/dev/ttyKONDO',
            'baudrate': 1500000,
            'publish_frequency': 50,
            'motor_list': ['{"id": 0, "model": "B3M-SC-1170-A", "name": "joint0", "direction": true, "offset": 0}',
                           '{"id": 1, "model": "B3M-SB-1040-A"}',
                           '{"id": 2, "name": "joint2"}',
                           '{"id": 3, "direction": false}',
                           '{"id": 4, "offset": 0.5}',
                           '{"id": 5}'],
        }]
    )

    ld = LaunchDescription()
    ld.add_action(kondo_b3m_ros2_node)

    return ld
