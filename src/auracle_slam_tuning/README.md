# auracle_slam_tuning

Offline harness for tuning `slam_toolbox`'s parameters with Bayesian
optimization (Optuna) instead of hand-guessing, scored against real ground
truth pulled from Gazebo and your gamefield STL.

## What changed in the rest of the workspace

Two small patches were made so this harness has ground truth to score
against - see `PATCH_NOTES.md` at the repo root for the exact diffs:

1. **`auracle_description/urdf/gt_odom.xacro`** (new file, included from
   `robot.urdf.xacro`) - adds a zero-noise Gazebo odometry plugin. This is
   a side channel only: it never publishes to ROS `/tf` and cannot
   interfere with your EKF's `odom->base_footprint` broadcast or with
   Nav2/slam_toolbox. It exists purely so trials have a "true pose" to
   score the SLAM estimate against.
2. **`auracle_bringup/launch/launch_sim.launch.py`** - added a `headless`
   launch arg (`headless:=true` runs `gzserver` only, no GUI, which is
   most of your wall-clock win when running 50-100 trials) and bridged
   the new ground-truth odom topic into ROS.

Neither patch changes what actually ships on the robot or during normal
autonomy runs - `gt_odom.xacro`'s plugin only exists in `sim_mode`.

**Before trusting any of this**, launch once manually and confirm the
ground truth topic actually shows up as named:
```bash
ros2 launch auracle_bringup launch_sim.launch.py headless:=true
# in another terminal:
gz topic -l | grep -i odom
ros2 topic echo /model/my_bot/odometry_gt --once
```
gz-sim's exact topic naming for the odometry-publisher plugin varies a
little across versions (Fortress/Harmonic/Jetty) - if it's not exactly
`/model/my_bot/odometry_gt`, fix the bridge line in
`launch_sim.launch.py` and the `odom_topic` default param in
`record_waypoints.py`, `course_driver.py`, `gt_trajectory_logger.py`.

## Install dependencies

```bash
pip install --break-system-packages optuna evo trimesh shapely scipy pillow pyyaml
```

## One-time setup (per gamefield layout)

These two only need to be redone if the course geometry changes.

### 1. Record the reference course path

Drive the *entire* course once by joystick/teleop in Gazebo - screen stop,
dock stop, ramp, dynamic obstacle zone, barrier/ramp choice, offload zone:

```bash
ros2 launch auracle_bringup launch_sim.launch.py    # GUI on, so you can drive it
# in another terminal, before you start driving:
ros2 run auracle_slam_tuning record_waypoints --ros-args \
    -p output_file:=/tmp/course_waypoints.yaml
# drive the full course now, Ctrl+C the recorder when you reach the offload zone
```

Every trial from here on replays this exact path via `course_driver.py`,
so you're comparing SLAM parameters against each other on identical runs,
not partly measuring path variance.

### 2. Build the ground-truth map from the STL

```bash
cd scripts
python3 rasterize_gt_map.py \
    --stl ../../auracle_description/models/gamefield/meshes/Gamefield.stl \
    --out-prefix /tmp/gt_map \
    --resolution 0.05
```

Open `/tmp/gt_map.pgm` in an image viewer (or load it in rviz as a Map)
and sanity-check it actually looks like your gamefield's walls. If it's
empty or looks like the wrong slice, adjust `--z-min`/`--z-max` - see the
comments at the top of `rasterize_gt_map.py` for what these need to line
up with (your lidar's actual world-frame mount height, not just the
`laser_joint` offset in the URDF).

## Running trials

One manual trial first, to debug the launch/teardown plumbing before
handing it to Optuna (things like Gazebo startup timing are very machine
dependent - `SETTLE_SECONDS` in `run_trial.py` almost certainly needs
adjusting for your machine):

```bash
cd scripts
python3 run_trial.py --trial-id manual0 \
    --waypoints /tmp/course_waypoints.yaml \
    --gt-map-prefix /tmp/gt_map
```

This should print a JSON result with `score`, `traj_rmse`, `map_iou`. If
it prints a `reason` field with `score: 10.0` instead, something in the
launch chain failed or timed out - check `/tmp/slam_trials/` for the
intermediate files.

Once that works, hand it to Optuna:

```bash
python3 optuna_study.py \
    --waypoints /tmp/course_waypoints.yaml \
    --gt-map-prefix /tmp/gt_map \
    --n-trials 60
```

This is resumable - `--study-name` controls the sqlite file, so running
it again with a larger `--n-trials` continues the same study rather than
starting over. At the end it prints the best params found and (if
matplotlib is installed) saves parameter-importance and optimization-
history plots.

## What's NOT handled here (on purpose)

- **Active/adaptive SLAM exploration policy** (slow down when uncertain,
  actively revisit a loop-closure candidate) - that's a genuine RL
  research problem, not a param-tuning one. Worth doing once static
  tuning here plateaus, not before.
- **Parallelizing trials** - `optuna_study.py --n-jobs` is wired up but
  will NOT work out of the box with more than 1 job, since every trial
  currently assumes it's the only Gazebo instance running (fixed ports,
  fixed `/tmp` file names). Parallelizing safely needs per-worker
  namespacing/ports; only take this on once single-trial runs are solid
  and you actually need the wall-clock speedup.
- **Robustness across course *variants*** - right now every trial drives
  the exact same recorded path against the exact same static gamefield.
  That's intentional for isolating SLAM parameter quality, but it means
  the tuned params are only proven for this one path. If you want params
  that generalize, the natural next step is recording 2-3 different
  paths/course arrangements and averaging the score across them per
  trial.
