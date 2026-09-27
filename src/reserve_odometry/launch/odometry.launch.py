from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import json
import math

def start(context):
    mode=LaunchConfiguration('map_mode').perform(context)
    config=Path(get_package_share_directory('reserve_odometry'))/'config'
    if mode=='calibrated':paths=[str(config/'route_loop.csv'),str(config/'route_hairpin_hypothesis.csv')];closed=True
    elif mode=='single':paths=[str(config/'route_loop.csv')];closed=True
    elif mode=='official':paths=[str(config/'route_0.csv'),str(config/'route_1.csv')];closed=False
    elif mode=='none':paths=[];closed=False
    else:raise ValueError('map_mode must be calibrated, single, official, or none')
    output_frame=LaunchConfiguration('output_frame').perform(context)
    if output_frame not in ('map','enu'):raise ValueError('output_frame must be map or enu')
    calibration=json.loads((config/'map_calibration.json').read_text())
    rotation=calibration['enu_to_map_R'];translation=calibration['enu_to_map_translation']+[calibration['map_z_minus_enu_up']]
    params={'use_sim_time':LaunchConfiguration('use_sim_time').perform(context).lower()=='true',
        'wheel_divisor':float(LaunchConfiguration('wheel_divisor').perform(context)),
        'speed_scale':float(LaunchConfiguration('speed_scale').perform(context)),
        'align_initial_position':LaunchConfiguration('align_initial_position').perform(context).lower()=='true',
        'initial_gnss_seconds':float(LaunchConfiguration('initial_gnss_seconds').perform(context)),
        'route_files':paths,'closed_route':closed,'compensate_velocity_lever_arm':False,
        'sparse_gnss_correction':LaunchConfiguration('sparse_gnss_correction').perform(context).lower()=='true',
        'gnss_xy_residual_correction':LaunchConfiguration('gnss_xy_residual_correction').perform(context).lower()=='true',
        'adhesion_accel_limit':float(LaunchConfiguration('adhesion_accel_limit').perform(context)),
        'adhesion_decel_limit':float(LaunchConfiguration('adhesion_decel_limit').perform(context)),
        'slip_release_rate':float(LaunchConfiguration('slip_release_rate').perform(context)),
        'slip_release_hold':float(LaunchConfiguration('slip_release_hold').perform(context)),
        'slip_max_latch':float(LaunchConfiguration('slip_max_latch').perform(context)),
        'offmap_departure':float(LaunchConfiguration('offmap_departure').perform(context)),
        'offmap_angle':float(LaunchConfiguration('offmap_angle').perform(context)),
        'offmap_min_span':float(LaunchConfiguration('offmap_min_span').perform(context)),
        'offmap_horizon':float(LaunchConfiguration('offmap_horizon').perform(context)),
        'offmap_max_distance':float(LaunchConfiguration('offmap_max_distance').perform(context)),
        'body_velocity_output':LaunchConfiguration('body_velocity_output').perform(context).lower()=='true','map_output':output_frame=='map','frame_id':'map' if output_frame=='map' else 'map_enu',
        'map_rotation_yaw':math.atan2(rotation[1][0],rotation[0][0]),'map_translation':translation,
        'publish_relative_position':mode=='none',
        'output_point_x':float(LaunchConfiguration('output_point_x').perform(context)),
        'output_point_z':float(LaunchConfiguration('output_point_z').perform(context))}
    return [Node(package='reserve_odometry',executable='reserve_odometry_node',output='screen',parameters=[params])]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('use_sim_time',default_value='true'),
        DeclareLaunchArgument('map_mode',default_value='calibrated'),
        DeclareLaunchArgument('output_frame',default_value='map'),
        DeclareLaunchArgument('sparse_gnss_correction',default_value='true'),
        DeclareLaunchArgument('gnss_xy_residual_correction',default_value='true'),
        DeclareLaunchArgument('adhesion_accel_limit',default_value='0.0'),
        DeclareLaunchArgument('adhesion_decel_limit',default_value='3.2'),
        DeclareLaunchArgument('slip_release_rate',default_value='0.2'),
        DeclareLaunchArgument('slip_release_hold',default_value='3.0'),
        DeclareLaunchArgument('slip_max_latch',default_value='10.0'),
        DeclareLaunchArgument('offmap_departure',default_value='10.0',description='Departure threshold in metres; 0 disables the off-map bridge'),
        DeclareLaunchArgument('offmap_angle',default_value='10.0',description='Minimum course disagreement in degrees'),
        DeclareLaunchArgument('offmap_min_span',default_value='5.0',description='Minimum course evidence span in metres'),
        DeclareLaunchArgument('offmap_horizon',default_value='3.0',description='Course evidence history in seconds'),
        DeclareLaunchArgument('offmap_max_distance',default_value='100.0',description='Maximum bridge distance without an accepted GNSS anchor'),
        DeclareLaunchArgument('body_velocity_output',default_value='false'),
        DeclareLaunchArgument('wheel_divisor',default_value='3.6'),
        DeclareLaunchArgument('speed_scale',default_value='1.0003350854241781'),
        DeclareLaunchArgument('align_initial_position',default_value='false'),
        DeclareLaunchArgument('initial_gnss_seconds',default_value='3.0'),
        DeclareLaunchArgument('output_point_x',default_value='0.0'),
        DeclareLaunchArgument('output_point_z',default_value='0.0'),OpaqueFunction(function=start)])
