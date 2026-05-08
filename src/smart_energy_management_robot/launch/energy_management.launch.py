import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('smart_energy_management_robot')

    plansys2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('plansys2_bringup'),
                'launch', 
                'plansys2_bringup_launch_monolithic.py'
            )
        ),
        launch_arguments={
            'model_file': os.path.join(pkg, 'pddl', 'domain.pddl'),
        }.items()
    )

    executor = Node(
        package='smart_energy_management_robot',
        executable='visit_action_executor.py',
        name='visit_action_executor',
        output='screen'
    )

    problem = Node(
        package='smart_energy_management_robot',
        executable='problem_generator.py',
        name='problem_generator',
        output='screen'
    )

    return LaunchDescription([
        plansys2,
        executor,
        problem,
    ])