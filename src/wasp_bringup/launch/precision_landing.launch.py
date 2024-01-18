import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch_ros.actions import Node
import xacro
from pathlib import Path


GAZEBO_PACKAGE_NAME = "wasp_gazebo"
DESCRIPTION_PACKAGE_NAME = "wasp_description"
WORLD = "precision_landing.world"



def generate_launch_description():
    pkg_share = get_package_share_directory(GAZEBO_PACKAGE_NAME)
    pkg_description = get_package_share_directory(DESCRIPTION_PACKAGE_NAME)
    gazebo_pkg = get_package_share_directory("gazebo_ros")

    xacro_file = Path(pkg_description).joinpath("urdf", "uav.xacro").as_posix()
    urdf = xacro.process_file(xacro_file).toxml()

    # params = {"robot_description": urdf, "use_sim_time": True}
    # node_robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="screen",
    #     parameters=[params],
    # )

    # source /usr/share/gazebo/setup.sh
    resources = [
        os.path.join(pkg_share, "worlds"),
        os.path.join(pkg_description, "meshes"),
        os.path.join(pkg_description, "models"),
    ]

    resource_env = AppendEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH", value=":".join(resources)
    )

    models = [os.path.join(pkg_description, "models")]
    models_env = AppendEnvironmentVariable(
        name="GAZEBO_MODEL_PATH", value=":".join(models)
    )

    plugins = [
        os.path.join(
            get_package_prefix(GAZEBO_PACKAGE_NAME), "lib", GAZEBO_PACKAGE_NAME
        ),
        os.path.join(pkg_share, "bin")
    ]

    plugins_env = AppendEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH", value=":".join(plugins)
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(gazebo_pkg, "launch", "gazebo.launch.py")]
        ),
        launch_arguments={"pause": "true","verbose": "true", "world": WORLD}.items(),
    )

    # spawn_entity = Node(
    #     package="gazebo_ros",
    #     executable="spawn_entity.py",
    #     arguments=["-topic", "robot_description", "-entity", "uav"],
    #     output="screen",
    # )

    ld = LaunchDescription()

    ld.add_action(plugins_env)
    ld.add_action(resource_env)
    ld.add_action(models_env)
    # ld.add_action(node_robot_state_publisher)
    ld.add_action(gazebo)
    # ld.add_action(spawn_entity)
    return ld
