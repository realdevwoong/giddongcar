"""PC-side Nav2 in two isolated DDS domains, with a shared web dashboard."""
import os
from pathlib import Path
import tempfile
import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def start(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    domains = [int(value('robot1_domain')), int(value('robot2_domain'))]
    if domains[0] == domains[1] or any(d < 0 or d > 232 for d in domains):
        raise RuntimeError('Robot domains must be distinct integers between 0 and 232')
    sim = value('use_sim_time').lower()
    if sim not in ('true', 'false'):
        raise RuntimeError('use_sim_time은 true 또는 false여야 합니다')
    sim = sim == 'true'
    # 시뮬 Nav2가 실물 로봇에 cmd_vel을 보내는 사고를 막는다. 실물용 DDS 설정이 남아 있으면 시작하지 않는다.
    if sim and (os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'
                or os.environ.get('ROS_STATIC_PEERS')
                or os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE')
                or os.environ.get('ROS_DISCOVERY_SERVER')):
        raise RuntimeError('Gazebo 모드는 이 PC 안에서만 통신해야 합니다: '
                           'export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; '
                           'unset ROS_STATIC_PEERS FASTRTPS_DEFAULT_PROFILES_FILE ROS_DISCOVERY_SERVER')
    map_path = Path(value('map')).expanduser().resolve()
    with map_path.open() as stream:
        map_config = yaml.safe_load(stream)
    if not (map_path.parent / map_config['image']).is_file():
        raise RuntimeError('Map image referenced by the YAML does not exist')
    params = Path(value('params_file')).expanduser().resolve()
    with params.open() as stream:
        config = yaml.safe_load(stream)
    # Require an explicit initial pose for each robot instead of assuming both are at (0, 0).
    amcl = config['amcl']['ros__parameters']
    amcl['set_initial_pose'] = False
    amcl.pop('initial_pose', None)
    # Nav2 기본값(60초) 안에 초기 위치를 못 받으면 costmap 활성화가 실패하고 되살아나지 않는다. 사람이 찍을 때까지 기다린다.
    for costmap in ('global_costmap', 'local_costmap'):
        config.setdefault(costmap, {}).setdefault(costmap, {}).setdefault('ros__parameters', {})[
            'initial_transform_timeout'] = 600.0
    temp = tempfile.TemporaryDirectory(prefix='pinky-fleet-')
    generated = Path(temp.name) / 'nav2_params.yaml'
    generated.write_text(yaml.safe_dump(config, sort_keys=False))
    processes = []
    for domain in domains:
        processes.append(ExecuteProcess(
            cmd=['ros2', 'launch', 'pinky_navigation', 'bringup_launch.xml',
                 f'map:={map_path}', f'params_file:={generated}']
                + (['use_sim_time:=True'] if sim else []),
            additional_env={'ROS_DOMAIN_ID': str(domain)}, output='screen'))
    dashboard = Path(get_package_prefix('pinky_fleet')) / 'lib' / 'pinky_fleet' / 'fleet_dashboard'
    processes.append(ExecuteProcess(
        cmd=[str(dashboard),
             '--robot1-domain', str(domains[0]), '--robot2-domain', str(domains[1]),
             '--host', value('host'), '--port', value('port')]
            + (['--use-sim-time'] if sim else []), output='screen'))
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=p, on_exit=[EmitEvent(event=Shutdown(reason='A fleet process exited'))]))
        for p in processes]
    def cleanup(context):
        temp.cleanup()
        return []

    handlers.append(RegisterEventHandler(OnShutdown(
        on_shutdown=[OpaqueFunction(function=cleanup)])))
    return handlers + processes


def generate_launch_description():
    pinky = Path(get_package_share_directory('pinky_navigation'))
    fleet = Path(get_package_share_directory('pinky_fleet'))
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(fleet / 'maps' / 'good3.yaml')),
        DeclareLaunchArgument('params_file', default_value=str(pinky / 'params/nav2_params.yaml')),
        DeclareLaunchArgument('robot1_domain', default_value='15'),
        DeclareLaunchArgument('robot2_domain', default_value='17'),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true: Gazebo /clock 시간으로 Nav2와 대시보드를 돌린다'),
        OpaqueFunction(function=start),
    ])
