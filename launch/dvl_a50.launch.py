import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    launch.actions.DeclareLaunchArgument('dvl_address', default_value='192.168.2.95'),
    dvl_a50 = launch_ros.actions.Node(
        package='dvl_a50', 
        executable='dvl_a50_sensor', 
        parameters=[{'dvl_address': launch.substitutions.LaunchConfiguration('dvl_address')}],
        output='screen')


    return launch.LaunchDescription([
        dvl_a50,
    ])


       
