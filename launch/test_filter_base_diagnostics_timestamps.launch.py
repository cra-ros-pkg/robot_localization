from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    parameters_file_dir = pathlib.Path(__file__).resolve().parent
    parameters_file_path = parameters_file_dir / 'test_filter_base_diagnostics_timestamps.yaml'    
    os.environ['FILE_PATH'] = str(parameters_file_dir)
    return LaunchDescription([
	
	 #*****test_filter_base_diagnostics_timestamps.test***** 
	launch_ros.actions.Node(
            package='robot_localization', node_executable='se_node', node_name='ekf_localization',
	    output='screen',
            parameters=[
                parameters_file_path,
                str(parameters_file_path),
                [EnvironmentVariable(name='FILE_PATH'), os.sep, 'test_filter_base_diagnostics_timestamps.yaml'],
           ],
	    ),
        launch_ros.actions.Node(
            package='robot_localization', node_executable='test_filter_base_diagnostics_timestamps',node_name='test_filter_base_diagnostics',
            output='screen',
        parameters=[
                parameters_file_path,
                str(parameters_file_path),
                [EnvironmentVariable(name='FILE_PATH'), os.sep, 'test_filter_base_diagnostics_timestamps.yaml'],
           ],
	    ),
])













