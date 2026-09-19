#!/usr/bin/env python3
"""
optuna_study.py

Bayesian-optimizes slam_toolbox's parameters against the composite score
from run_trial.py (trajectory RMSE + 1-IoU on the STL ground truth map).

Start narrow. The params below are the ones most responsible for loop
closures either fixing or wrecking the map (see the ranges' comments) -
resist the urge to throw all ~20 slam_toolbox params in at once; Optuna
will waste trials exploring dimensions that don't matter and you'll need
proportionally more trials to converge.

Usage:
    # one-time setup, see README.md for the full sequence
    python3 optuna_study.py --waypoints /tmp/course_waypoints.yaml \
        --gt-map-prefix /tmp/gt_map --n-trials 60

Persisted to sqlite, so this is resumable:
    python3 optuna_study.py --study-name slam_tune --n-trials 40   # run more later

Requires: optuna
    pip install optuna --break-system-packages
"""
import argparse
import os
import sys

import optuna

sys.path.insert(0, os.path.dirname(__file__))
from run_trial import run_trial  # noqa: E402


def suggest_params(trial: optuna.Trial):
    return {
        # How far the robot must move before a new scan is added. Too
        # small = redundant scans slow everything down and inflate scan
        # buffer error; too large = missed detail and worse loop closure
        # candidates.
        'minimum_travel_distance': trial.suggest_float('minimum_travel_distance', 0.10, 0.45),
        'minimum_travel_heading': trial.suggest_float('minimum_travel_heading', 0.10, 0.45),

        # How far back slam_toolbox will look for a loop closure candidate.
        # This is usually the biggest lever on "loop closure ruins the map":
        # too large invites false-positive closures that snap unrelated
        # parts of the map together.
        'loop_search_maximum_distance': trial.suggest_float(
            'loop_search_maximum_distance', 2.0, 8.0),

        # How strict a loop closure candidate must be to get accepted.
        # Raising these trades fewer (but more trustworthy) loop closures
        # against a map that stays more open (uncorrected) longer.
        'loop_match_minimum_response_coarse': trial.suggest_float(
            'loop_match_minimum_response_coarse', 0.35, 0.60),
        'loop_match_minimum_response_fine': trial.suggest_float(
            'loop_match_minimum_response_fine', 0.45, 0.70),
        'link_match_minimum_response_fine': trial.suggest_float(
            'link_match_minimum_response_fine', 0.05, 0.30),

        # Local scan-matching search window - too small and it can't
        # recover from odometry error between scans; too large and it's
        # slower and more prone to matching the wrong feature.
        'correlation_search_space_dimension': trial.suggest_float(
            'correlation_search_space_dimension', 0.3, 0.8),

        # How much the scan matcher trusts distance vs angle agreement
        # between scans.
        'distance_variance_penalty': trial.suggest_float(
            'distance_variance_penalty', 0.3, 0.8),
        'angle_variance_penalty': trial.suggest_float(
            'angle_variance_penalty', 0.5, 1.5),
    }


def make_objective(waypoints_file, gt_map_prefix, workdir):
    def objective(trial: optuna.Trial):
        params = suggest_params(trial)
        result = run_trial(
            params, trial_id=f't{trial.number}',
            waypoints_file=waypoints_file, gt_map_prefix=gt_map_prefix,
            workdir=workdir,
        )
        # Surface everything for later inspection (study.trials_dataframe())
        for k, v in result.items():
            if k != 'score':
                trial.set_user_attr(k, v)
        return result['score']
    return objective


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--waypoints', required=True)
    ap.add_argument('--gt-map-prefix', required=True)
    ap.add_argument('--n-trials', type=int, default=60)
    ap.add_argument('--study-name', default='slam_tune')
    ap.add_argument('--storage', default=None,
                     help='defaults to sqlite:///<study-name>.db next to this script')
    ap.add_argument('--workdir', default='/tmp/slam_trials')
    ap.add_argument('--n-jobs', type=int, default=1,
                     help='parallel trials; only raise this if you have separate '
                          'Gazebo instances/ports configured per worker')
    args = ap.parse_args()

    storage = args.storage or f"sqlite:///{args.study_name}.db"
    study = optuna.create_study(
        study_name=args.study_name, storage=storage,
        load_if_exists=True, direction='minimize',
    )

    study.optimize(
        make_objective(args.waypoints, args.gt_map_prefix, args.workdir),
        n_trials=args.n_trials, n_jobs=args.n_jobs,
    )

    print("\nBest trial:")
    print(f"  score: {study.best_trial.value}")
    print(f"  params: {study.best_trial.params}")
    print(f"  user_attrs: {study.best_trial.user_attrs}")

    try:
        import optuna.visualization.matplotlib as vis
        fig = vis.plot_param_importances(study)
        fig.figure.savefig(f"{args.study_name}_importances.png")
        fig2 = vis.plot_optimization_history(study)
        fig2.figure.savefig(f"{args.study_name}_history.png")
        print(f"\nSaved {args.study_name}_importances.png and _history.png")
    except ImportError:
        print("\n(matplotlib not installed - skipping importance/history plots)")


if __name__ == '__main__':
    main()
