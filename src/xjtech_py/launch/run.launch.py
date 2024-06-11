from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
def generate_launch_description():
    # 创建一个包含参数加载器的节点
    config_path = PathJoinSubstitution([FindPackageShare('xjtech_py'), 'config', 'config.yaml'])
    parameter = ParameterFile(
        config_path,
        allow_substs=True
    )
    xjtech_node_with_parameters = Node(
        name='xjtech_node',
        package='xjtech_py',
        executable='xjtech_node',
        namespace = "xjtech",
        # output="screen",
        parameters=[parameter]
        )
    
    return LaunchDescription([
        xjtech_node_with_parameters,
    ])