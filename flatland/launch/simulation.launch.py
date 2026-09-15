"""Standalone warehouse simulation using installed package assets."""
import atexit
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

ROBOTS = ['turtlebot', 'castor_wheeled_differential_robot', 'omni_drive_robot', 'forklift']


def setup(context):
    share = Path(get_package_share_directory('flatland'))
    value = lambda key: LaunchConfiguration(key).perform(context)
    robot, namespace = value('robot'), value('robot_namespace').strip('/')
    localization, drive_model = value('localization'), value('drive_model')
    if robot not in ROBOTS:
        raise ValueError(f'robot must be one of {ROBOTS}')
    if localization not in ['ground_truth', 'external']:
        raise ValueError('localization must be ground_truth or external')
    if drive_model not in ['physics', 'identified'] or (drive_model == 'identified' and robot != 'forklift'):
        raise ValueError('drive_model:=identified is available for the forklift example')
    initial = [float(value(key)) for key in ['x', 'y', 'yaw']]
    runtime = tempfile.TemporaryDirectory(prefix='flatland2_')
    atexit.register(runtime.cleanup)
    directory = Path(runtime.name)
    model = yaml.safe_load((share/'robot'/f'{robot}.model.yaml').read_text())
    for plugin in model['plugins']:
        if plugin['type'] == 'ModelTfPublisher':
            plugin['publish_tf_world'] = False
        if plugin['type'] == 'MockLidar3D':
            plugin['pcd_path'] = str(share/'maps/warehouse/warehouse.pcd')
    if drive_model == 'identified':
        model['plugins'][0] = dict(type='SystemIdDrive', name='identified_drive', body='base_link',
            model_path=str(share/'config/forklift_dynamics.yaml'), command_sub='ackermann_cmd',
            odom_pub='odom', ground_truth_pub='ground_truth/odom', ground_truth_pose_pub='ground_truth/pose',
            odom_frame_id='odom', ground_truth_frame_id='map', pub_rate=30.0, cmd_timeout=0.5)
    if value('terrain').lower() == 'true':
        model['plugins'].insert(1, yaml.safe_load((share/'config/terrain.yaml').read_text()))
    model_path = directory/'robot.model.yaml'
    model_path.write_text(yaml.safe_dump(model))
    world = yaml.safe_load((share/'maps/warehouse/warehouse.world.yaml').read_text())
    world['layers'][0]['map'] = str(share/'maps/warehouse/warehouse.yaml')
    world['models'] = [dict(name='robot', namespace=namespace, model=str(model_path), pose=initial)]
    if value('objects').lower() == 'true':
        for obj in yaml.safe_load((share/'config/objects.yaml').read_text()):
            world['models'].append(dict(name=obj['name'], namespace=obj['name'], pose=obj['pose'],
                model=str(share/'objects'/f"{obj['model']}.model.yaml")))
    world_path = directory/'world.yaml'
    world_path.write_text(yaml.safe_dump(world))
    server = Node(package='flatland_server', executable='flatland_server', name='flatland_server',
        output='screen', parameters=[dict(use_sim_time=True, world_path=str(world_path),
        update_rate=100.0, step_size=0.01, show_viz=True, viz_pub_rate=15.0, default_extrude_height=0.0)])
    nodes = [server,
        RegisterEventHandler(OnProcessExit(target_action=server,
            on_exit=[EmitEvent(event=Shutdown(reason='Flatland server exited'))])),
        Node(package='flatland', executable='odometry_tf.py', namespace=namespace,
            parameters=[dict(use_sim_time=True, publish_map_tf=localization == 'ground_truth',
                             x=initial[0], y=initial[1], yaw=initial[2])])]
    if value('publish_map').lower() == 'true':
        nodes.extend([Node(package='nav2_map_server', executable='map_server', name='map_server',
            parameters=[dict(use_sim_time=True, yaml_filename=str(share/'maps/warehouse/warehouse.yaml'))]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='map_lifecycle',
            parameters=[dict(use_sim_time=True, autostart=True, node_names=['map_server'])])])
    if robot == 'omni_drive_robot':
        nodes.append(Node(package='flatland', executable='omni_command.py', namespace=namespace,
                          parameters=[dict(use_sim_time=True)]))
    if value('show_viz').lower() == 'true':
        nodes.append(Node(package='flatland_viz', executable='flatland_viz',
            arguments=['-d', str(share/'rviz/flatland.rviz')], parameters=[dict(use_sim_time=True)]))
    return nodes


def generate_launch_description():
    defaults = dict(robot='turtlebot', robot_namespace='', show_viz='false',
                    localization='ground_truth', drive_model='physics', objects='true',
                    terrain='false', publish_map='true', x='0.0', y='-2.0', yaw='0.0')
    return LaunchDescription([DeclareLaunchArgument(key, default_value=value)
                              for key, value in defaults.items()] + [OpaqueFunction(function=setup)])
