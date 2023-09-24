from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()
    pkg_name = "rviz2_shape"
    path = FindPackageShare(pkg_name)
    default_model_path = PathJoinSubstitution(['urdf', '01-myfirst.urdf'])
    default_rviz_config_path = PathJoinSubstitution([path, 'rviz', 'urdf.rviz'])

    # These parameters are maintained for backwards compatibility
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    ld.add_action(gui_arg)
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)

    # This parameter has changed its meaning slightly from previous versions
    ld.add_action(DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                        description='Path to robot urdf file relative to urdf_tutorial package'))
    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_tutorial'), 'launch', 'display.launch.py']),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'rvizconfig': LaunchConfiguration('rvizconfig'),
            'gui': LaunchConfiguration('gui')}.items()
    ))

    return ld