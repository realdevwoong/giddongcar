"""PC-side Nav2 in two isolated DDS domains, with a shared web dashboard."""
from pathlib import Path
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

ROOT = Path(__file__).resolve().parent


def start(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    domains = [int(value('robot1_domain')), int(value('robot2_domain'))]
    if domains[0] == domains[1] or any(d < 0 or d > 232 for d in domains):
        raise RuntimeError('Robot domains must be distinct integers between 0 and 232')
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
    temp = tempfile.TemporaryDirectory(prefix='pinky-fleet-')
    generated = Path(temp.name) / 'nav2_params.yaml'
    generated.write_text(yaml.safe_dump(config, sort_keys=False))
    processes = []
    for domain in domains:
        processes.append(ExecuteProcess(
            cmd=['ros2', 'launch', 'pinky_navigation', 'bringup_launch.xml',
                 f'map:={map_path}', f'params_file:={generated}'],
            additional_env={'ROS_DOMAIN_ID': str(domain)}, output='screen'))
    processes.append(ExecuteProcess(
        cmd=['/usr/bin/python3', str(ROOT / 'fleet_dashboard.py'),
             '--robot1-domain', str(domains[0]), '--robot2-domain', str(domains[1]),
             '--host', value('host'), '--port', value('port')], output='screen'))
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
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(ROOT / 'good.yaml')),
        DeclareLaunchArgument('params_file', default_value=str(pinky / 'params/nav2_params.yaml')),
        DeclareLaunchArgument('robot1_domain', default_value='15'),
        DeclareLaunchArgument('robot2_domain', default_value='17'),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='8080'),
        OpaqueFunction(function=start),
    ])
