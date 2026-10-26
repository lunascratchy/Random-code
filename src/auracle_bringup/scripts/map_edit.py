#!/usr/bin/env python3
"""Look at or clean a saved map (map_saver .yaml + .pgm). Coordinates are metres in the map frame,
the same numbers RViz shows (Publish Point, or the planner log "plan from (x, y)").

Show the area around a point (# wall, . free, blank unknown, + marks the point):
    ros2 run auracle_bringup map_edit.py ~/maps/trial_2.yaml --show 0.08 -0.03 --radius 0.6

Erase walls inside boxes (xmin xmax ymin ymax) into a NEW map; the original is never changed:
    ros2 run auracle_bringup map_edit.py ~/maps/trial_2.yaml --out ~/maps/trial_2_clean \\
        --erase 0.14 0.22 -0.30 0.22 --erase -0.20 0.13 0.14 0.22

Check that a robot of a given half-width can drive between two points:
    ros2 run auracle_bringup map_edit.py ~/maps/trial_2_clean.yaml --path 0.08 -0.03 0.63 -0.77
"""
import argparse, math, os, re
from collections import deque


def load(yaml_path):
    meta = {}
    for line in open(yaml_path):
        if ':' in line:
            k, v = line.split(':', 1)
            meta[k.strip()] = v.strip()
    pgm = meta['image']
    if not os.path.isabs(pgm):
        pgm = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), pgm)
    data = open(pgm, 'rb').read()
    m = re.match(rb'P5\s+(?:#.*\s+)*(\d+)\s+(\d+)\s+(\d+)\s', data)
    w, h = int(m[1]), int(m[2])
    origin = [float(v) for v in meta['origin'].strip('[]').split(',')]
    return dict(meta=meta, w=w, h=h, res=float(meta['resolution']), ox=origin[0], oy=origin[1],
                header=data[:m.end()], px=bytearray(data[m.end():]), yaml_text=open(yaml_path).read(), pgm=pgm)


def cell(M, x, y):
    return int(round((x - M['ox']) / M['res'])), M['h'] - 1 - int(round((y - M['oy']) / M['res']))


def is_wall(M, cx, cy):
    return M['px'][cy * M['w'] + cx] < 50


def show(M, x, y, radius):
    n = int(radius / M['res'])
    cx0, cy0 = cell(M, x, y)
    print(f"around ({x}, {y}), {M['res'] * 100:.0f} cm per character, up = +y, right = +x")
    for cy in range(cy0 - n, cy0 + n + 1):
        row = ''
        for cx in range(cx0 - n, cx0 + n + 1):
            if not (0 <= cx < M['w'] and 0 <= cy < M['h']):
                row += ' '
            elif (cx, cy) == (cx0, cy0):
                row += '+'
            else:
                v = M['px'][cy * M['w'] + cx]
                row += '#' if v < 50 else ('.' if v > 250 else ' ')
        print(row)


def erase(M, xa, xb, ya, yb):
    x0, y1 = cell(M, xa, yb)
    x1, y0 = cell(M, xb, ya)
    n = 0
    for cy in range(max(0, y1), min(M['h'], y0 + 1)):
        for cx in range(max(0, x0), min(M['w'], x1 + 1)):
            if is_wall(M, cx, cy):
                M['px'][cy * M['w'] + cx] = 254
                n += 1
    return n


def path(M, sx, sy, gx, gy, half_width):
    w, h = M['w'], M['h']
    r = int(math.ceil(half_width / M['res']))
    offs = [(i, j) for i in range(-r, r + 1) for j in range(-r, r + 1) if i * i + j * j <= r * r]
    blocked = bytearray(w * h)
    for cy in range(h):
        for cx in range(w):
            if is_wall(M, cx, cy):
                for i, j in offs:
                    if 0 <= cx + i < w and 0 <= cy + j < h:
                        blocked[(cy + j) * w + cx + i] = 1
    S, G = cell(M, sx, sy), cell(M, gx, gy)
    if blocked[S[1] * w + S[0]]:
        return "start is inside a wall (or closer than half_width to one)"
    if blocked[G[1] * w + G[0]]:
        return "goal is inside a wall (or closer than half_width to one)"
    seen = {S}
    q = deque([S])
    while q:
        c = q.popleft()
        if c == G:
            return "path exists"
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (c[0] + dx, c[1] + dy)
            if 0 <= n[0] < w and 0 <= n[1] < h and n not in seen and not blocked[n[1] * w + n[0]]:
                seen.add(n)
                q.append(n)
    return f"NO path (only {len(seen) * M['res'] ** 2:.1f} m2 reachable from the start)"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('yaml')
    ap.add_argument('--show', nargs=2, type=float, metavar=('X', 'Y'))
    ap.add_argument('--radius', type=float, default=0.6)
    ap.add_argument('--erase', nargs=4, type=float, action='append', metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'))
    ap.add_argument('--out', help='new map name without extension, e.g. ~/maps/trial_2_clean')
    ap.add_argument('--path', nargs=4, type=float, metavar=('SX', 'SY', 'GX', 'GY'))
    ap.add_argument('--half-width', type=float, default=0.13, help='robot half-width in m (footprint 0.26 wide)')
    a = ap.parse_args()

    M = load(os.path.expanduser(a.yaml))
    if a.erase:
        if not a.out:
            ap.error('--erase needs --out (the original map is never overwritten)')
        for box in a.erase:
            print(f"erase x {box[0]}..{box[1]}, y {box[2]}..{box[3]}: {erase(M, *box)} wall cells")
        out = os.path.expanduser(a.out)
        open(out + '.pgm', 'wb').write(M['header'] + bytes(M['px']))
        open(out + '.yaml', 'w').write(M['yaml_text'].replace(M['meta']['image'], os.path.basename(out) + '.pgm'))
        print(f"wrote {out}.yaml / .pgm")
    if a.show:
        show(M, *a.show, a.radius)
    if a.path:
        print(f"half-width {a.half_width} m: {path(M, *a.path, a.half_width)}")


if __name__ == '__main__':
    main()
