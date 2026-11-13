"""Opt-in hardware bringup with a single guarded command path.

hardware:=false launches only the mux, guard, converter and output watchdog.
Their motor/servo outputs are isolated under /safety/test; no device is opened.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool_arg(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ('true', 'false', '1', '0'):
        raise ValueError(f'{name} must be true or false, got {value!r}')
    return value in ('true', '1')


def _nodes(context):
    hardware = _bool_arg(context, 'hardware')
    bench = _bool_arg(context, 'bench')
    recover = _bool_arg(context, 'recover')
    joystick = _bool_arg(context, 'enable_joystick')
    vesc_config = LaunchConfiguration('vesc_config').perform(context)
    guard_config = LaunchConfiguration('guard_config').perform(context)
    with open(guard_config, encoding='utf-8') as stream:
        guard_params = yaml.safe_load(stream)['collision_guard']['ros__parameters']
    max_speed = float(guard_params['max_speed'])
    test_outputs = [] if hardware else [
        ('/commands/motor/speed', '/safety/test/motor_speed'),
        ('/commands/servo/position', '/safety/test/servo_position'),
    ]

    mux = Node(
        package='ackermann_mux', executable='ackermann_mux', name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')], output='screen',
        # The actual installed mux publishes ackermann_cmd, not ackermann_cmd_out.
        remappings=[('ackermann_cmd', '/safety/raw_drive')])
    guard = Node(
        package='race_stack', executable='collision_guard', name='collision_guard',
        output='screen', parameters=[guard_config, {
            'input_topic': '/safety/raw_drive', 'output_topic': '/ackermann_cmd',
            'wheelbase': float(LaunchConfiguration('wheelbase').perform(context)),
            'max_steer': float(LaunchConfiguration('max_steer').perform(context)),
            'odom_topic': LaunchConfiguration('odom_topic').perform(context),
            'bench': bench,
            'recovery_enabled': recover,
        }])
    converter = Node(
        package='vesc_ackermann', executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node', output='screen', parameters=[vesc_config],
        remappings=[('commands/motor/speed', '/safety/motor_request'),
                    ('commands/servo/position', '/safety/servo_request')])
    watchdog = Node(
        package='race_stack', executable='collision_motor_watchdog',
        name='collision_motor_watchdog', output='screen',
        parameters=[vesc_config, {'command_timeout': 0.2, 'rearm_hold': 0.2,
                                  'control_rate': 50.0, 'max_speed': max_speed}],
        remappings=test_outputs)

    stopping = False

    def stop_before_shutdown(event, launch_context):
        nonlocal stopping
        if stopping or launch_context.is_shutdown:
            return []
        stopping = True
        reason = f'guarded command process exited: {event.action.name}'
        # Keep the driver alive while the helper repeatedly sends emergency stop.
        helper = Node(
            package='race_stack', executable='collision_stop', name='collision_stop',
            output='screen', remappings=test_outputs)
        return [
            LogInfo(msg=reason + '; sending stop before shutting down'),
            RegisterEventHandler(OnProcessExit(
                target_action=helper,
                on_exit=[EmitEvent(event=Shutdown(reason=reason))])),
            helper,
        ]

    critical = [mux, guard, converter, watchdog]
    actions = [RegisterEventHandler(OnProcessExit(
        target_action=node, on_exit=stop_before_shutdown)) for node in critical]
    actions.extend(critical)
    if hardware:
        if joystick:
            # The outer race launch owns joystick_teleop. Only provide /joy here.
            actions.append(Node(
                package='joy', executable='joy_node', name='joy', output='screen',
                parameters=[{'device_id': 0, 'deadzone': 0.05,
                             'autorepeat_rate': 20.0}]))
        actions.extend([
            Node(package='vesc_ackermann', executable='vesc_to_odom_node',
                 name='vesc_to_odom_node', parameters=[vesc_config]),
            Node(package='vesc_driver', executable='vesc_driver_node',
                 name='vesc_driver_node', parameters=[vesc_config]),
            Node(package='urg_node', executable='urg_node_driver', name='urg_node',
                 parameters=[LaunchConfiguration('sensors_config')]),
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name='static_baselink_to_laser',
                 arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0',
                            'base_link', 'laser']),
        ])
    return actions


def generate_launch_description():
    stack = get_package_share_directory('f1tenth_stack')
    race = get_package_share_directory('race_stack')
    defaults = {
        'hardware': 'true',
        'bench': 'false',
        'recover': 'true',
        'enable_joystick': 'true',
        'wheelbase': '0.33',
        'max_steer': '0.4796',
        'odom_topic': '/odom_imu',
        'vesc_config': os.path.join(stack, 'config', 'vesc.yaml'),
        'sensors_config': os.path.join(stack, 'config', 'sensors.yaml'),
        'mux_config': os.path.join(stack, 'config', 'mux.yaml'),
        'guard_config': os.path.join(race, 'config', 'collision_guard.yaml'),
    }
    return LaunchDescription([
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ] + [OpaqueFunction(function=_nodes)])
