# PX4 example
# ROS_ARGS : -s libgazebo_ros_init.so -s libgazebo_ros_factory.so
# GZ_COMMAND : gzserver /home/lwx/code/PX4-Autopilot/Tools/simulation/gazebo-classic/
#               sitl_gazebo-classic/worlds/empty.world 
# -s libgazebo_ros_init.so -s libgazebo_ros_factory.so
# Using: /home/lwx/code/PX4-Autopilot/Tools/simulation/gazebo-classic
#               /sitl_gazebo-classic/models/iris/iris.sdf
# GZ_MODEL_COMMAND : gz model --verbose 
# --spawn-file=/home/lwx/code/PX4-Autopilot/Tools/simulation/gazebo-classic/
# sitl_gazebo-classic/models/iris/iris.sdf 
# --model-name=iris -x 1.01 -y 0.98 -z 0.83
# PX4 Launch:model and world
# export PX4_SIM_MODEL=gazebo-classic_${model}
# export PX4_SIM_WORLD=${world}
# PX4 Envitonment:find binary,model,world and plugin file
# export PX4_HOME=~/code/PX4-Autopilot
# export BUILD_DIR=$PX4_HOME/build/px4_sitl_default
# export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$SRC_DIR/Tools/simulation/gazebo-classic/
#                           sitl_gazebo-classic/models:~/gazebo/model_editor_models
# export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/gazebo/plugins/lib
#                           :$BUILD_DIR/build_gazebo-classic
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$BUILD_DIR/build_gazebo-classic
import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, 
    IncludeLaunchDescription, 
    RegisterEventHandler,
    SetEnvironmentVariable
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions.launch_configuration import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    model_name = "hyper_robot4"
    world_name = "empty"
    gz_lib = LaunchConfiguration("gz_lib",default="--verbose")
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), 
                                               '/gazebo.launch.py']),
                launch_arguments={"extra_gazebo_args":gz_lib}.items()
             )
    gazebo_ros2_control_demos_path = os.path.join(
        get_package_share_directory('gazebo_robot'))

    xacro_file = os.path.join(gazebo_ros2_control_demos_path,
                              'model',
                              model_name,
                              model_name + '.xacro.urdf')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()} # type: ignore

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', model_name],
                        output='screen')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )
    #traj
    # load_joint_trajectory_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
    #          'joint_trajectory_controller'],
    #     output='screen'
    # )
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_position_controller'],
        output='screen'
    )
    
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])
