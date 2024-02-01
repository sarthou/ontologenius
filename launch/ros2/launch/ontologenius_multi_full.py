import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    # args that can be set from the command line or a default will be used
    language_arg = DeclareLaunchArgument(
        "language", default_value=TextSubstitution(text="en")
    )
    intern_file_arg = DeclareLaunchArgument(
        "intern_file", default_value=TextSubstitution(text=os.path.join(get_package_share_directory('ontologenius'), "file_intern/ontologenius.owl"))
    )
    config_file_arg = DeclareLaunchArgument(
        "config_file", default_value=TextSubstitution(text="none")
    )
    display_arg = DeclareLaunchArgument(
        "display", default_value=TextSubstitution(text="true")
    )
    files_arg = DeclareLaunchArgument(
        "files", default_value=TextSubstitution(text = os.path.join(get_package_share_directory('ontologenius'), "files/attribute.owl") + " " +
                                                       os.path.join(get_package_share_directory('ontologenius'), "files/measure.owl") + " " +
                                                       os.path.join(get_package_share_directory('ontologenius'), "files/property.owl") + " " +
                                                       os.path.join(get_package_share_directory('ontologenius'), "files/positionProperty.owl") + " " +
                                                       os.path.join(get_package_share_directory('ontologenius'), "files/testIndividuals.owl"))
    )
    robot_file_arg = DeclareLaunchArgument(
        "robot_file", default_value=TextSubstitution(text="none")
    )
    human_file_arg = DeclareLaunchArgument(
        "human_file", default_value=TextSubstitution(text="none")
    )
    tcmalloc_path_arg = DeclareLaunchArgument(
        "tcmalloc_path", default_value=TextSubstitution(text="")
    )

    ontologenius_core_node = Node(
            package='ontologenius',
            executable='ontologenius_multi',
            name='ontologenius_core',
            output='screen',
            arguments=['-l', LaunchConfiguration('language'),
                       '-c', LaunchConfiguration('config_file'),
                       '-i', LaunchConfiguration('intern_file'),
                       '-r', LaunchConfiguration('robot_file'),
                       '-h', LaunchConfiguration('human_file'),
                       '-d', LaunchConfiguration('display'),
                       LaunchConfiguration('files')]
        )
    
    ontologenius_gui_node = Node(
            package='ontologenius',
            executable='ontoloGUI',
            name='ontologenius_gui',
            output='screen'
        )

    return LaunchDescription([
        language_arg,
        intern_file_arg,
        config_file_arg,
        display_arg,
        files_arg,
        robot_file_arg,
        human_file_arg,
        tcmalloc_path_arg,
        ontologenius_core_node,
        ontologenius_gui_node
    ])