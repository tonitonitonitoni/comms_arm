from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('aligned_arm')
    urdf_path = os.path.join(pkg_share, 'urdf', 'aligned_arm.urdf')
    #urdf_path = os.path.join(pkg_share, 'urdf', 'gripper.urdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={'verbose': 'false'}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_path).read()
        }]
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'aligned_arm',
            '-topic', 'robot_description',
            # '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    image_display = Node(
        package='lazy_arm',
        executable='image_display',
        name='image_display',
        output='screen'
    )
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        image_display
    ])
