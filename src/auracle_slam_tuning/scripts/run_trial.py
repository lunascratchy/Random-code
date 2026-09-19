#!/usr/bin/env python3
"""
run_trial.py

One full Optuna trial = one full mapping run:

    render slam params -> launch headless Gazebo+robot -> launch slam_toolbox
    -> launch gt_trajectory_logger -> run course_driver until it finishes
    (or timeout) -> save the map -> tear everything down cleanly
    -> score trajectory (evo APE) + score map (align_and_score_map.py)
    -> return one combined number for Optuna to minimize

This is deliberately a plain script using subprocess, not a ROS node -
it needs to start and stop entire ROS graphs per trial, which isn't
something a single node can do to itself.

You almost certainly need to adjust:
    - SETTLE_SECONDS: how long Gazebo/controllers need before slam_toolbox
      and the course driver should start (currently a blind sleep; a
      topic-poll wait would be more robust but this is simpler to debug)
    - TRIAL_TIMEOUT_SECONDS: how long one course run is allowed to take
      before being treated as a failed trial
    - the ROS package/launch file names if you rename anything

Run standalone for one manual trial (useful for debugging before handing
this to Optuna):
    python3 run_trial.py --trial-id manual0 \
        --waypoints /tmp/course_waypoints.yaml \
        --gt-map-prefix /tmp/gt_map
"""
import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time
import zipfile

sys.path.insert(0, os.path.dirname(__file__))
from align_and_score_map import score_map  # noqa: E402

TEMPLATE_PATH = os.path.join(
    os.path.dirname(__file__), '..', 'config', 'slam_params_template.yaml'
)

SETTLE_SECONDS = 12
TRIAL_TIMEOUT_SECONDS = 240
FAILURE_SCORE = 10.0  # returned to Optuna for a trial that crashed/timed out

# How the two components combine into one number Optuna minimizes.
TRAJ_WEIGHT = 1.0
MAP_WEIGHT = 1.0


def render_params(params, out_path):
    with open(TEMPLATE_PATH) as f:
        template = f.read()
    with open(out_path, 'w') as f:
        f.write(template.format(**params))


def start(cmd, **kwargs):
    """Start a subprocess in its own process group so we can kill the
    whole tree it spawns (ros2 launch fans out into many child procs)."""
    return subprocess.Popen(
        cmd, preexec_fn=os.setsid, stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL, **kwargs
    )


def stop(proc, grace=5):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=grace)
    except (subprocess.TimeoutExpired, ProcessLookupError):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


def hard_cleanup():
    """Safety net between trials - orphaned gz/ros processes will corrupt
    the next trial's results (e.g. a leftover gzserver holding the port)."""
    for pattern in ('gz sim', 'gzserver', 'slam_toolbox', 'course_driver',
                     'gt_trajectory_logger', 'ros2 launch'):
        subprocess.run(['pkill', '-9', '-f', pattern],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)


def run_evo_ape(gt_file, est_file, results_zip):
    if os.path.exists(results_zip):
        os.remove(results_zip)
    result = subprocess.run(
        ['evo_ape', 'tum', gt_file, est_file, '-a', '--save_results', results_zip],
        capture_output=True, text=True
    )
    if not os.path.exists(results_zip):
        raise RuntimeError(
            f"evo_ape did not produce results (stdout/stderr below).\n"
            f"STDOUT: {result.stdout}\nSTDERR: {result.stderr}"
        )
    with zipfile.ZipFile(results_zip) as z:
        with z.open('stats.json') as f:
            stats = json.load(f)
    return stats['rmse']


def run_trial(params, trial_id, waypoints_file, gt_map_prefix, workdir='/tmp/slam_trials'):
    os.makedirs(workdir, exist_ok=True)
    slam_params_path = os.path.join(workdir, f'slam_params_{trial_id}.yaml')
    gt_traj_path = os.path.join(workdir, f'traj_gt_{trial_id}.tum')
    est_traj_path = os.path.join(workdir, f'traj_est_{trial_id}.tum')
    map_prefix = os.path.join(workdir, f'map_{trial_id}')
    evo_zip_path = os.path.join(workdir, f'evo_{trial_id}.zip')

    render_params(params, slam_params_path)

    procs = {}
    try:
        procs['gz'] = start([
            'ros2', 'launch', 'auracle_bringup', 'launch_sim.launch.py',
            'headless:=true'
        ])
        time.sleep(SETTLE_SECONDS)

        procs['slam'] = start([
            'ros2', 'launch', 'auracle_bringup', 'online_async_launch.py',
            f'params_file:={slam_params_path}', 'use_sim_time:=true'
        ])
        time.sleep(3)

        procs['logger'] = start([
            'ros2', 'run', 'auracle_slam_tuning', 'gt_trajectory_logger',
            '--ros-args',
            '-p', f'gt_out:={gt_traj_path}',
            '-p', f'est_out:={est_traj_path}',
        ])

        driver = start([
            'ros2', 'run', 'auracle_slam_tuning', 'course_driver',
            '--ros-args', '-p', f'waypoints_file:={waypoints_file}',
        ])
        try:
            ret = driver.wait(timeout=TRIAL_TIMEOUT_SECONDS)
            course_ok = (ret == 0)
        except subprocess.TimeoutExpired:
            course_ok = False
            stop(driver)

        stop(procs.pop('logger'))  # flush TUM files before killing slam_toolbox

        if not course_ok:
            return {'score': FAILURE_SCORE, 'reason': 'course_timeout_or_error'}

        subprocess.run([
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', map_prefix, '--ros-args', '-p', 'use_sim_time:=true'
        ], timeout=30)

    except Exception as e:
        return {'score': FAILURE_SCORE, 'reason': f'exception: {e}'}
    finally:
        for p in procs.values():
            stop(p)
        hard_cleanup()

    if not os.path.exists(f'{map_prefix}.yaml'):
        return {'score': FAILURE_SCORE, 'reason': 'map_saver_failed'}

    try:
        traj_rmse = run_evo_ape(gt_traj_path, est_traj_path, evo_zip_path)
    except Exception as e:
        return {'score': FAILURE_SCORE, 'reason': f'evo_failed: {e}'}

    try:
        map_result = score_map(f'{map_prefix}.yaml', gt_map_prefix)
    except Exception as e:
        return {'score': FAILURE_SCORE, 'reason': f'map_scoring_failed: {e}'}

    composite = TRAJ_WEIGHT * traj_rmse + MAP_WEIGHT * map_result['score']
    return {
        'score': composite,
        'traj_rmse': traj_rmse,
        'map_iou': map_result['iou'],
        'map_precision': map_result['precision'],
        'map_recall': map_result['recall'],
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--trial-id', default='manual0')
    ap.add_argument('--waypoints', required=True)
    ap.add_argument('--gt-map-prefix', required=True)
    args = ap.parse_args()

    baseline_params = {
        'minimum_travel_distance': 0.20,
        'minimum_travel_heading': 0.20,
        'loop_search_maximum_distance': 1.0,
        'correlation_search_space_dimension': 0.5,
        'loop_match_minimum_response_coarse': 0.55,
        'loop_match_minimum_response_fine': 0.65,
        'link_match_minimum_response_fine': 0.1,
        'distance_variance_penalty': 0.5,
        'angle_variance_penalty': 1.0,
    }

    result = run_trial(baseline_params, args.trial_id, args.waypoints, args.gt_map_prefix)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
