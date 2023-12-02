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
# Not implement yet
# Command:
# export PX4_SIM_MODEL=gazebo-classic_hyper_robot4
# ${BUILD_DIR}/bin/px4 ${BUILD_DIR}/etc
import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, 
    SetEnvironmentVariable
)
from launch.substitutions import EnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions.launch_configuration import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    
    model_name = "gazebo-classic_hyper_robot4"
    world_name = "empty.world"
    px4_build = "/home/lwx/code/PX4-Autopilot/build/px4_sitl_default/"
    set_model_env = SetEnvironmentVariable(name="PX4_SIM_MODEL",value=model_name)
    # model_env = EnvironmentVariable(name="PX4_SIM_MODEL",default_value=model_name)
    # world_env = EnvironmentVariable(name="PX4_SIM_world",default_value=world_name)
    px4_launch = ExecuteProcess(
        cmd=[px4_build + "bin/px4",
             px4_build + "etc"],
        shell=True,
        output='screen',
        emulate_tty=True
    )
    return LaunchDescription([
        set_model_env,
        px4_launch
    ])