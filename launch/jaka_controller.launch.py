from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('robot_tool')
    
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'robot_config.yaml'),
        description='Path to the robot configuration file'
    )
    
    # 创建JAKA控制器节点
    jaka_controller_node = Node(
        package='robot_tool',
        executable='jaka_controller_node',
        name='jaka_controller',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('robot_state_dual', '/robot_state_dual'),
            ('multi_movj', '/multi_movj'),
            ('servo_joint_command', '/servo_joint_command'),
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        jaka_controller_node,
    ]) 