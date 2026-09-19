#!/usr/bin/env python3
"""
align_and_score_map.py

Compares a slam_toolbox-generated map (saved via map_saver_cli, so a
.pgm + .yaml pair) against the STL-derived ground truth raster from
rasterize_gt_map.py (.npy + .json), and returns an IoU-based score.

Why the alignment search:
    rasterize_gt_map.py's world->map transform is only an approximation
    (see its docstring). Rather than trust that it's pixel-perfect, this
    script brute-forces a small window of (dx, dy, dtheta) and keeps
    whichever alignment maximizes agreement between the two occupied-cell
    sets. This also makes the score robust to slam_toolbox trials that
    place the map frame slightly differently trial-to-trial.

Can be run standalone for a sanity check:
    python3 align_and_score_map.py \
        --slam-map /tmp/trial_map.yaml \
        --gt-prefix /tmp/gt_map
or imported as a module:
    from align_and_score_map import score_map
    result = score_map(slam_map_yaml, gt_prefix)
    result['iou']  # higher is better
    result['score']  # 1 - iou, i.e. LOWER is better (what Optuna minimizes)

Requires: numpy, pillow, pyyaml, scipy
"""
import argparse
import json

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import shift as nd_shift, rotate as nd_rotate
from scipy.signal import fftconvolve


def load_slam_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img = np.array(Image.open(yaml_path.rsplit('/', 1)[0] + '/' + meta['image']))
    if img.ndim == 3:
        img = img[..., 0]
    img = np.flipud(img)  # undo map_saver's row-flip convention
    occ_thresh = meta.get('occupied_thresh', 0.65) * 255
    if meta.get('negate', 0):
        occupied = img > occ_thresh
    else:
        occupied = img < (255 - occ_thresh)
    return occupied, meta['resolution'], meta['origin'][0], meta['origin'][1]


def load_gt_map(prefix):
    grid = np.load(f"{prefix}.npy")
    with open(f"{prefix}.json") as f:
        meta = json.load(f)
    return grid, meta['resolution'], meta['origin_x'], meta['origin_y']


def resample_to_resolution(grid, src_res, dst_res):
    if abs(src_res - dst_res) < 1e-6:
        return grid
    from scipy.ndimage import zoom
    factor = src_res / dst_res
    return zoom(grid.astype(float), factor, order=0) > 0.5


def pad_to_common_shape(a, b):
    h = max(a.shape[0], b.shape[0])
    w = max(a.shape[1], b.shape[1])
    pa = np.zeros((h, w), dtype=bool)
    pb = np.zeros((h, w), dtype=bool)
    pa[:a.shape[0], :a.shape[1]] = a
    pb[:b.shape[0], :b.shape[1]] = b
    return pa, pb


def iou(a, b):
    inter = np.logical_and(a, b).sum()
    union = np.logical_or(a, b).sum()
    return inter / union if union > 0 else 0.0


def best_translation(rotated_occ, gt_occ, max_shift_cells):
    """
    Exact integer-cell translation search via FFT cross-correlation:
    correlation[dy, dx] = number of cells where gt is occupied AND
    rotated_occ shifted by (dy, dx) is occupied. This is the same
    quantity a naive nested dx/dy loop would compute, just done with an
    FFT instead of ~1000 python-level nd_shift calls - which matters once
    maps are a few hundred cells per side rather than our 100x120 test grid.
    """
    a = gt_occ.astype(float)
    b = rotated_occ.astype(float)
    # 'full' correlation via convolution with the flipped kernel
    corr = fftconvolve(a, b[::-1, ::-1], mode='full')
    center_r, center_c = b.shape[0] - 1, b.shape[1] - 1

    r0 = max(0, center_r - max_shift_cells)
    r1 = min(corr.shape[0], center_r + max_shift_cells + 1)
    c0 = max(0, center_c - max_shift_cells)
    c1 = min(corr.shape[1], center_c + max_shift_cells + 1)
    window = corr[r0:r1, c0:c1]

    rel_r, rel_c = np.unravel_index(np.argmax(window), window.shape)
    dy = (r0 + rel_r) - center_r
    dx = (c0 + rel_c) - center_c
    return int(dy), int(dx)


def best_alignment(slam_occ, gt_occ, max_shift_cells=15, max_rot_deg=8, rot_step=2):
    """Search rotation explicitly (few candidates); for each, solve the
    translation exactly and cheaply via FFT cross-correlation, then keep
    whichever rotation+translation pair gives the best IoU."""
    best = {'iou': -1.0, 'dx': 0, 'dy': 0, 'dtheta': 0.0}
    for dtheta in np.arange(-max_rot_deg, max_rot_deg + 1e-9, rot_step):
        rotated = nd_rotate(slam_occ.astype(float), dtheta, reshape=False, order=0) > 0.5
        dy, dx = best_translation(rotated, gt_occ, max_shift_cells)
        shifted = nd_shift(rotated.astype(float), (dy, dx), order=0) > 0.5
        score = iou(shifted, gt_occ)
        if score > best['iou']:
            best.update({'iou': score, 'dx': dx, 'dy': dy, 'dtheta': dtheta})
    return best


def score_map(slam_map_yaml, gt_prefix):
    slam_occ, slam_res, _, _ = load_slam_map(slam_map_yaml)
    gt_occ, gt_res, _, _ = load_gt_map(gt_prefix)

    # Bring both grids to the same resolution before comparing
    slam_occ = resample_to_resolution(slam_occ, slam_res, gt_res)
    slam_occ, gt_occ = pad_to_common_shape(slam_occ, gt_occ)

    best = best_alignment(slam_occ, gt_occ)
    precision = np.logical_and(
        nd_shift(nd_rotate(slam_occ.astype(float), best['dtheta'], reshape=False, order=0),
                 (best['dy'], best['dx']), order=0) > 0.5,
        gt_occ
    ).sum() / max(1, (nd_rotate(slam_occ.astype(float), best['dtheta'], reshape=False,
                                 order=0) > 0.5).sum())
    recall = np.logical_and(
        nd_shift(nd_rotate(slam_occ.astype(float), best['dtheta'], reshape=False, order=0),
                 (best['dy'], best['dx']), order=0) > 0.5,
        gt_occ
    ).sum() / max(1, gt_occ.sum())

    return {
        'iou': best['iou'],
        'precision': precision,
        'recall': recall,
        'alignment_dx_cells': best['dx'],
        'alignment_dy_cells': best['dy'],
        'alignment_dtheta_deg': best['dtheta'],
        'score': 1.0 - best['iou'],  # what Optuna should MINIMIZE
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--slam-map', required=True, help='.yaml from map_saver_cli')
    ap.add_argument('--gt-prefix', required=True, help='prefix used with rasterize_gt_map.py')
    args = ap.parse_args()

    result = score_map(args.slam_map, args.gt_prefix)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
