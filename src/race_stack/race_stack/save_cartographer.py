"""Finish a mapping trajectory and save matching pbstream / PGM / YAML files."""
import argparse
import math
from pathlib import Path
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from cartographer_ros_msgs.msg import SubmapList, TrajectoryStates
from cartographer_ros_msgs.srv import FinishTrajectory, GetTrajectoryStates, WriteState


def call(node, service_type, name, request, timeout=120.0):
    client = node.create_client(service_type, name)
    try:
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'{name}: service unavailable; keep Cartographer running.')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError(f'{name}: timed out; inspect mapping terminal before retrying.')
        result = future.result()
        if result is None or result.status.code != 0:
            raise RuntimeError(f'{name}: {result}')
        return result
    finally:
        node.destroy_client(client)


def save(node, base, resolution):
    # Refuse overwrites before finishing a trajectory or writing anything.
    outputs = [Path(str(base) + ext) for ext in ('.pbstream', '.pgm', '.yaml')]
    for path in outputs:
        if path.exists():
            raise RuntimeError(f'Already exists: {path}. Choose a new map name.')
    converter = Path(get_package_prefix('cartographer_ros')) / 'lib/cartographer_ros/cartographer_pbstream_to_ros_map'
    if not converter.is_file():
        raise RuntimeError(f'Missing converter: {converter}')

    observed = []
    sub = node.create_subscription(SubmapList, '/submap_list', observed.append, 1)
    try:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not any(m.submap for m in observed):
            rclpy.spin_once(node, timeout_sec=0.2)
        if not any(m.submap for m in observed):
            raise RuntimeError('No submaps received. Check /scan, /odom_imu and TF first.')
    finally:
        node.destroy_subscription(sub)

    states = call(node, GetTrajectoryStates, '/get_trajectory_states',
                  GetTrajectoryStates.Request()).trajectory_states
    trajectories = list(zip(states.trajectory_id, states.trajectory_state))
    usable = [(tid, state) for tid, state in trajectories
              if state in (TrajectoryStates.ACTIVE, TrajectoryStates.FINISHED)]
    if len(usable) != 1:
        raise RuntimeError(f'Expected one mapping trajectory, got {trajectories}.')
    base.parent.mkdir(parents=True, exist_ok=True)
    tid, state = usable[0]
    if state == TrajectoryStates.ACTIVE:
        print(f'Finishing trajectory {tid}; mapping stops here.', flush=True)
        call(node, FinishTrajectory, '/finish_trajectory',
             FinishTrajectory.Request(trajectory_id=tid))

    pbstream = outputs[0]
    call(node, WriteState, '/write_state', WriteState.Request(
        filename=str(pbstream), include_unfinished_submaps=True))
    if not pbstream.is_file() or pbstream.stat().st_size == 0:
        raise RuntimeError(f'State file was not written locally: {pbstream}')
    print(f'Saved {pbstream}', flush=True)
    # Render from the saved state, avoiding a stale /map during loop closure.
    command = [str(converter), f'-pbstream_filename={pbstream}',
               f'-map_filestem={base}', f'-resolution={resolution}']
    try:
        subprocess.run(command, check=True, timeout=180)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError(
            f'PGM/YAML conversion failed; pbstream is retained at {pbstream}. '
            f'Retry with cartographer_pbstream_to_ros_map using that file: {exc}') from exc
    for path in outputs[1:]:
        if not path.is_file() or path.stat().st_size == 0:
            raise RuntimeError(f'Map output missing or empty: {path}')
    print(f'Saved: {base}.pbstream / .pgm / .yaml')
    print('You may now stop the mapping terminal with Ctrl+C.')
    print('Race configuration unchanged. Use localization.mode=cartographer with '
          'this pbstream (or AMCL with its PGM/YAML). No slam_toolbox posegraph was created.')


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', help='Absolute or relative map path without extension')
    parser.add_argument('--resolution', type=float, default=0.05)
    opts = parser.parse_args(args)
    if not math.isfinite(opts.resolution) or opts.resolution <= 0:
        parser.error('resolution must be positive and finite')
    base = Path(opts.output).expanduser().resolve()
    rclpy.init(args=[])
    node = rclpy.create_node('save_cartographer')
    try:
        save(node, base, opts.resolution)
    except (RuntimeError, OSError) as exc:
        parser.exit(1, f'Save failed: {exc}\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
