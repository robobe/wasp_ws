from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

PACKAGE = "wasp_bringup"
WORLD = "gimbal.world"

def generate_launch_description():
    ld = LaunchDescription()

    share = get_package_share_directory(PACKAGE)
    pkg = get_package_prefix(PACKAGE)
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    verbose = LaunchConfiguration("verbose")
    arg_gazebo_verbose = DeclareLaunchArgument("verbose", default_value="true")
    world = LaunchConfiguration("world")
    arg_gazebo_world = DeclareLaunchArgument("world", default_value=WORLD)


    resources = [os.path.join(share, "worlds")]

    resource_env = AppendEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH", value=":".join(resources)
    )

    models = [os.path.join(share, "models")]

    models_env = AppendEnvironmentVariable(
        name="GAZEBO_MODEL_PATH", value=":".join(models)
    )

    plugins = [os.path.join(share, "bin"), os.path.join(pkg, "lib")]

    plugins_env = AppendEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH", value=":".join(plugins)
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    gazebo_pkg, 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'verbose': verbose, "world": world}.items()
             )



    ld.add_action(models_env)
    ld.add_action(plugins_env)
    ld.add_action(resource_env)
    ld.add_action(arg_gazebo_verbose)
    ld.add_action(arg_gazebo_world)
    ld.add_action(gazebo)
    return ld
