from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    kondo_b3m_node = Node(
        package='kondo_b3m_ros2',
        executable='kondo_b3m',
        remappings=[('b3m_joint_state', 'joint_states')],
        parameters=[{
            'port_name': '/dev/ttyKONDO',
            'baudrate': 1500000,
            'publish_frequency': 50,
        }]
    )

    kondo_b3m_util_0 = Node(
        package='kondo_b3m_ros2',
        executable='kondo_b3m_util',
        name='util0',
        parameters=[{
            'id': 0,
            'name': 'joint0',
            'mode': 'position'
        }]
    )

    kondo_b3m_util_1 = Node(
        package='kondo_b3m_ros2',
        executable='kondo_b3m_util',
        name='util1',
        parameters=[{
            'id': 1,
            'name': 'joint1',
            'mode': 'speed'
        }]
    )

    ld = LaunchDescription()
    ld.add_action(kondo_b3m_node)
    ld.add_action(kondo_b3m_util_0)
    ld.add_action(kondo_b3m_util_1)

    return ld
