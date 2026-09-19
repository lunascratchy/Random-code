#!/usr/bin/env python3
"""
rasterize_gt_map.py

Builds a ground-truth occupancy grid directly from the gamefield STL mesh,
so Optuna trials can be scored against known geometry instead of "does
this look okay in rviz".

Why a cross-section, not a top-down silhouette:
    Your 2D lidar only sees a thin horizontal slice of the world at its
    mount height. If we projected the ENTIRE mesh (floor to ceiling) down
    onto the XY plane, the ground-truth map would include walls/structure
    the lidar physically cannot see, and the IoU score would be unfairly
    low no matter how good slam_toolbox's params are. So instead we slice
    the mesh at the lidar's height band and rasterize THAT cross-section -
    it's the same information the 2D lidar is working with.

Frame handling:
    The STL's vertices are in mm (model.sdf scales by 0.001) and the mesh
    is placed in the model.sdf's <pose> in Gazebo WORLD frame. slam_toolbox's
    "map" frame is not the world frame - by convention it starts out
    coincident with wherever "odom" was at boot, which is wherever the
    robot was spawned. So: mesh_in_map_frame = R(-spawn_yaw) . T(-spawn_xy)
    . T(mesh_pose) . Scale(0.001) . mesh_in_stl_units

    This is an approximation (assumes odom hasn't drifted from map's
    origin convention, which holds well enough at t=0 for build-time
    ground truth). align_and_score_map.py does a further small
    brute-force search to correct any residual offset, so this doesn't
    need to be perfect - just close.

Defaults below are pulled directly from your model.sdf and
launch_sim.launch.py spawn_entity args. Override anything that's specific
to your actual course/robot via the CLI flags.

Usage:
    python3 rasterize_gt_map.py \
        --stl /path/to/Gamefield.stl \
        --out-prefix /tmp/gt_map \
        --resolution 0.05

Outputs:
    <out-prefix>.npy   - boolean occupancy array (True = occupied)
    <out-prefix>.json  - {resolution, origin_x, origin_y} metadata
    <out-prefix>.pgm + <out-prefix>.yaml - nav2 map_server-style files,
        purely so you can eyeball it in rviz (map_server / rviz Map panel)
        next to the slam_toolbox output.

Requires: trimesh, shapely, numpy, pillow
    pip install trimesh shapely numpy pillow --break-system-packages
"""
import argparse
import json

import numpy as np
import trimesh
from PIL import Image


def build_transform(mesh_pose_xyz, mesh_scale, spawn_xy, spawn_yaw):
    """4x4 transform: STL-local (mm) -> approximate SLAM map frame (m)."""
    S = np.eye(4)
    S[0, 0] = S[1, 1] = S[2, 2] = mesh_scale

    T_mesh = np.eye(4)
    T_mesh[:3, 3] = mesh_pose_xyz

    T_spawn = np.eye(4)
    T_spawn[:2, 3] = [-spawn_xy[0], -spawn_xy[1]]

    c, s = np.cos(-spawn_yaw), np.sin(-spawn_yaw)
    R_spawn = np.eye(4)
    R_spawn[0, 0], R_spawn[0, 1] = c, -s
    R_spawn[1, 0], R_spawn[1, 1] = s, c

    # Apply order: scale, then place in world, then shift/rotate into map frame
    return R_spawn @ T_spawn @ T_mesh @ S


def rasterize(mesh, z_min, z_max, resolution, margin=1.0):
    verts = mesh.vertices
    x_min, x_max = verts[:, 0].min() - margin, verts[:, 0].max() + margin
    y_min, y_max = verts[:, 1].min() - margin, verts[:, 1].max() + margin

    width = int(np.ceil((x_max - x_min) / resolution))
    height = int(np.ceil((y_max - y_min) / resolution))
    grid = np.zeros((height, width), dtype=bool)

    def world_to_cell(x, y):
        col = int((x - x_min) / resolution)
        row = int((y - y_min) / resolution)
        return row, col

    def draw_line(r0, c0, r1, c1):
        # Bresenham, thickened by 1 cell so thin walls don't vanish
        n = max(abs(r1 - r0), abs(c1 - c0), 1)
        for i in range(n + 1):
            r = int(round(r0 + (r1 - r0) * i / n))
            c = int(round(c0 + (c1 - c0) * i / n))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < height and 0 <= cc < width:
                        grid[rr, cc] = True

    # Slice the mesh at several heights within the lidar's band and
    # rasterize each cross-section's boundary segments.
    n_slices = max(3, int((z_max - z_min) / (resolution * 2)))
    any_slice = False
    for z in np.linspace(z_min, z_max, n_slices):
        section = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
        if section is None:
            continue
        any_slice = True
        planar, _ = section.to_2D()
        for entity in planar.entities:
            pts = planar.vertices[entity.points]
            for i in range(len(pts) - 1):
                r0, c0 = world_to_cell(*pts[i])
                r1, c1 = world_to_cell(*pts[i + 1])
                draw_line(r0, c0, r1, c1)

    if not any_slice:
        raise RuntimeError(
            "mesh.section() returned nothing in the given z range - "
            "check --z-min/--z-max against where the mesh actually sits "
            "after the world/spawn transform (print mesh.vertices[:,2] "
            "min/max to sanity check)."
        )

    origin_x, origin_y = x_min, y_min
    return grid, origin_x, origin_y


def save_map_server_files(grid, resolution, origin_x, origin_y, out_prefix):
    # nav2 map_server convention: image row 0 is the TOP, but map (0,0) is
    # bottom-left, so flip vertically on write.
    img = np.full(grid.shape, 254, dtype=np.uint8)  # free = white
    img[grid] = 0  # occupied = black
    img = np.flipud(img)
    Image.fromarray(img, mode='L').save(f"{out_prefix}.pgm")

    yaml_text = (
        f"image: {out_prefix.split('/')[-1]}.pgm\n"
        f"resolution: {resolution}\n"
        f"origin: [{origin_x}, {origin_y}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n"
    )
    with open(f"{out_prefix}.yaml", 'w') as f:
        f.write(yaml_text)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--stl', required=True, help='Path to Gamefield.stl')
    ap.add_argument('--out-prefix', required=True)
    ap.add_argument('--resolution', type=float, default=0.05,
                     help='Match slam_toolbox\'s "resolution" param for easy comparison')
    # Defaults below come straight from model.sdf's <pose> and <scale>
    ap.add_argument('--mesh-x', type=float, default=0.135886)
    ap.add_argument('--mesh-y', type=float, default=6.443587)
    ap.add_argument('--mesh-z', type=float, default=0.789877)
    ap.add_argument('--mesh-scale', type=float, default=0.001)
    # Defaults below come from spawn_entity's args in launch_sim.launch.py
    ap.add_argument('--spawn-x', type=float, default=-2.6)
    ap.add_argument('--spawn-y', type=float, default=-1.3)
    ap.add_argument('--spawn-yaw', type=float, default=1.57)
    # ADJUST THESE to your real lidar mount height above the world/mesh
    # z=0 plane once the robot is spawned. laser_joint puts the lidar at
    # z=0.085 above base_footprint, but base_footprint's own world height
    # depends on wheel radius/chassis clearance - verify in Gazebo (Ctrl
    # click the lidar link, read its world pose) before trusting this band.
    ap.add_argument('--z-min', type=float, default=0.10,
                     help='Lidar band lower bound in world Z, meters')
    ap.add_argument('--z-max', type=float, default=0.30,
                     help='Lidar band upper bound in world Z, meters')
    args = ap.parse_args()

    mesh = trimesh.load(args.stl)
    T = build_transform(
        mesh_pose_xyz=[args.mesh_x, args.mesh_y, args.mesh_z],
        mesh_scale=args.mesh_scale,
        spawn_xy=(args.spawn_x, args.spawn_y),
        spawn_yaw=args.spawn_yaw,
    )
    mesh.apply_transform(T)

    grid, origin_x, origin_y = rasterize(mesh, args.z_min, args.z_max, args.resolution)

    np.save(f"{args.out_prefix}.npy", grid)
    with open(f"{args.out_prefix}.json", 'w') as f:
        json.dump({'resolution': args.resolution,
                    'origin_x': origin_x, 'origin_y': origin_y}, f, indent=2)
    save_map_server_files(grid, args.resolution, origin_x, origin_y, args.out_prefix)

    occ = int(grid.sum())
    print(f"Wrote {args.out_prefix}.npy / .json / .pgm+.yaml  "
          f"({grid.shape[1]}x{grid.shape[0]} cells, {occ} occupied)")


if __name__ == '__main__':
    main()
