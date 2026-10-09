"""PC-side Nav2 in two isolated DDS domains, with a shared web dashboard."""
import copy
import math
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
from launch.substitutions import EnvironmentVariable

# 실물 로봇 도메인(robot1, robot2). launch 인자 기본값도 여기서 가져간다. 시뮬 Nav2는 이 도메인에 붙으면 안 된다.
REAL_DOMAINS = (15, 17)
# 실물용 DDS 설정. 시뮬에서 남아 있으면 이 PC 밖(실물 로봇)과 통신할 수 있다. sim.launch.py가 지우는 것과 같다.
REAL_DDS_ENV = ('ROS_STATIC_PEERS', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'ROS_DISCOVERY_SERVER',
                'ROS_SUPER_CLIENT', 'CYCLONEDDS_URI')


def parse_pose(text, name):
    """'x,y,yaw' -> (x, y, yaw). 비어 있으면 None: 사람이 대시보드에서 찍을 때까지 기다린다."""
    if not text.strip():
        return None
    try:
        pose = tuple(float(v) for v in text.split(','))
    except ValueError:
        pose = ()
    if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
        raise RuntimeError(f'{name}은 "x,y,yaw" 형식이어야 합니다: {text!r}')
    return pose


def start(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    domains = [int(value('robot1_domain')), int(value('robot2_domain'))]
    if domains[0] == domains[1] or any(d < 0 or d > 232 for d in domains):
        raise RuntimeError('Robot domains must be distinct integers between 0 and 232')
    poses = [parse_pose(value(f'robot{i}_initial_pose'), f'robot{i}_initial_pose') for i in (1, 2)]
    sim = value('use_sim_time').lower()
    if sim not in ('true', 'false'):
        raise RuntimeError('use_sim_time은 true 또는 false여야 합니다')
    sim = sim == 'true'
    auto_spin = value('auto_spin').lower()
    if auto_spin not in ('true', 'false'):
        raise RuntimeError('auto_spin은 true 또는 false여야 합니다')
    auto_spin = auto_spin == 'true'
    # 시뮬 Nav2가 실물 로봇에 cmd_vel을 보내는 사고를 막는다. 실물 도메인이거나 실물용 DDS 설정이 남아 있으면 시작하지 않는다.
    if sim and set(domains) & set(REAL_DOMAINS):
        raise RuntimeError(f'Gazebo 모드에서는 실물 로봇 도메인 {REAL_DOMAINS[0]}/{REAL_DOMAINS[1]}을 쓸 수 없습니다: '
                           'robot1_domain:=25 robot2_domain:=27로 띄우세요')
    if sim and (os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'
                or any(os.environ.get(key) for key in REAL_DDS_ENV)):
        raise RuntimeError('Gazebo 모드는 이 PC 안에서만 통신해야 합니다: '
                           'export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; unset ' + ' '.join(REAL_DDS_ENV))
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
    for i, (domain, pose) in enumerate(zip(domains, poses), 1):
        robot_params = generated
        if pose:
            # 시뮬처럼 로봇 위치를 이미 알면 AMCL이 켜지자마자 그 자리로 잡는다. 대시보드에서 다시 찍어도 된다.
            own = copy.deepcopy(config)
            own['amcl']['ros__parameters'].update(
                set_initial_pose=True, initial_pose=dict(x=pose[0], y=pose[1], z=0.0, yaw=pose[2]))
            robot_params = Path(temp.name) / f'nav2_params_robot{i}.yaml'
            robot_params.write_text(yaml.safe_dump(own, sort_keys=False))
        processes.append(ExecuteProcess(
            cmd=['ros2', 'launch', 'pinky_navigation', 'bringup_launch.xml',
                 f'map:={map_path}', f'params_file:={robot_params}']
                + (['use_sim_time:=True'] if sim else []),
            additional_env={'ROS_DOMAIN_ID': str(domain)}, output='screen'))
    dashboard = Path(get_package_prefix('pinky_fleet')) / 'lib' / 'pinky_fleet' / 'fleet_dashboard'
    processes.append(ExecuteProcess(
        cmd=[str(dashboard),
             '--robot1-domain', str(domains[0]), '--robot2-domain', str(domains[1]),
             '--robot1-camera-host', LaunchConfiguration('robot1_camera_host', default='').perform(context),
             '--robot2-camera-host', LaunchConfiguration('robot2_camera_host', default='').perform(context),
             '--camera-port', LaunchConfiguration('camera_port', default='5000').perform(context),
             '--host', value('host'), '--port', value('port'), '--map', str(map_path)]
            + (['--use-sim-time'] if sim else [])
            # 위치를 알려 준 로봇은 전역 위치 찾기를 하지 않는다. 나머지는 대시보드가 켜지자마자 스스로 찾는다
            + [arg for i, pose in enumerate(poses, 1) if pose for arg in ('--known-pose', f'robot{i}')]
            + (['--auto-spin'] if auto_spin else [])
            + (['--traffic-zones', value('traffic_zones')] if value('traffic_zones') else []), output='screen'))
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
    fleet = Path(get_package_share_directory('pinky_fleet'))
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(fleet / 'maps' / 'good3.yaml')),
        DeclareLaunchArgument('params_file', default_value=str(fleet / 'params/nav2_params.yaml')),   # 팀용 복사본
        DeclareLaunchArgument('robot1_domain', default_value=str(REAL_DOMAINS[0])),
        DeclareLaunchArgument('robot2_domain', default_value=str(REAL_DOMAINS[1])),
        DeclareLaunchArgument('robot1_camera_host', default_value=EnvironmentVariable('ROBOT1_IP', default_value=''),
                              description='실물 카메라 스트림을 받을 로봇1 주소. 비우면 카메라 연결을 끈다'),
        DeclareLaunchArgument('robot2_camera_host', default_value=EnvironmentVariable('ROBOT2_IP', default_value=''),
                              description='실물 카메라 스트림을 받을 로봇2 주소. 비우면 카메라 연결을 끈다'),
        DeclareLaunchArgument('camera_port', default_value='5000',
                              description='Pinky Pro HTTP MJPEG 카메라 포트'),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true: Gazebo /clock 시간으로 Nav2와 대시보드를 돌린다'),
        DeclareLaunchArgument('robot1_initial_pose', default_value='',
                              description='"x,y,yaw" (map 좌표). 비우면 대시보드가 전역 위치 찾기로 스스로 찾는다(틀리면 초기 위치를 찍는다)'),
        DeclareLaunchArgument('robot2_initial_pose', default_value=''),
        DeclareLaunchArgument('auto_spin', default_value='true',
                              description='true: 위치를 모르는 로봇은 AMCL이 켜지자마자 제자리에서 한 바퀴 돌며 위치를 찾는다'
                                          '(실물이 사람 확인 없이 움직인다. 끝나면 0 속도). false: 가만히 찾고 ⟳ 버튼으로만 돈다'),
        DeclareLaunchArgument('traffic_zones', default_value=str(fleet / 'params' / 'traffic_good3.yaml'),
                              description='교통 정리 구역 YAML(좁은 문에 한 대씩). 비우면(traffic_zones:=) 끈다. '
                                          '지도가 구역 파일과 다르면 대시보드가 스스로 끈다'),
        OpaqueFunction(function=start),
    ])
