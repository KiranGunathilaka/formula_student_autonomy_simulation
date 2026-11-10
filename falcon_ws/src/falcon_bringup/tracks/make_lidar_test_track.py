#!/usr/bin/env python3
"""
Build lidar_test_track.csv: a trackdrive lap to the FS-AI 2026 track rules, for the
lidar statistics (docs/lidar_statistics.html).

  python3 make_lidar_test_track.py            # writes lidar_test_track.csv here
  python3 make_lidar_test_track.py --plot out.png

Rules it keeps (FS-AI 2026 rules D8.1, p. 33, the same lines as autocross D6.1.1):
straights no longer than 80 m, constant turns up to 50 m diameter, hairpins at least
9 m outside diameter, track at least 3 m wide; cones at most 5 m apart in the driving
direction, closer in corners (FS-AI dynamic events set-up and cones specification).

The hairpins are 13 m outside diameter, not the 9 m minimum: the ADS-DV's tightest
kerb-to-kerb turn is 9.9 m (+-21 deg road-wheel angle, 1.533 m wheelbase), so it
cannot drive a 9 m hairpin at all, and on 11 m ones the sim's pure pursuit, at full
lock, still touched cones.

The lap, anticlockwise from the start line:
  60 m start straight -> hairpin -> 12 m -> hairpin the other way (an S of two
  hairpins) -> 30 m-diameter constant turn -> chicane -> 90 deg turn -> back straight
  -> two more hairpins back to back -> decreasing-radius turn onto the start straight.
Two straights (back straight, west side) are solved so the loop closes.
"""

import argparse
import math
import os

import numpy as np

WIDTH = 3.5                    # m, cone row to cone row
HAIRPIN_OD = 13.0              # m, outside diameter of each hairpin
R_HP = HAIRPIN_OD / 2 - WIDTH / 2
STRAIGHT_SPACING = 5.0         # m, the rule's maximum
CORNER_SPACING = 2.5           # m, along each cone row in corners
START_BEHIND = 6.0             # m, car starts this far behind the start line


def layout(x1, x2):
    """Segments: ('S', length[, label]) or ('A', signed angle deg (+ left), radius[, label])."""
    return [
        ('S', 60.0, 'start straight'),
        ('A', 180, R_HP, 'hairpin 1'),                # after the long straight
        ('S', 12.0),
        ('A', -180, R_HP, 'hairpin 2'),               # an S of two hairpins
        ('S', 20.0),
        ('A', 90, 15.0, 'constant turn'),             # 30 m diameter
        ('S', 8.0),
        ('A', 35, 10.0), ('A', -70, 10.0, 'chicane'), ('A', 35, 10.0),
        ('S', 5.0),
        ('A', 90, 10.0),
        ('S', x1, 'back straight'),                   # solved
        ('A', 180, R_HP, 'hairpin 3'),
        ('S', 10.0),
        ('A', -180, R_HP, 'hairpin 4'),               # back to back with hairpin 3
        ('S', 30.0),
        ('A', 90, 9.0),
        ('S', x2),                                    # west side (solved)
        ('A', 45, 18.0, 'decreasing radius'), ('A', 45, 9.0),
        ('S', 0.0),                                   # closes onto the start straight
    ]


def features():
    """(label, x, y) at the middle of each labelled part of the lap."""
    segs = layout(*solve())
    out = []
    for i, seg in enumerate(segs):
        label = seg[2] if seg[0] == 'S' and len(seg) > 2 else \
            seg[3] if seg[0] == 'A' and len(seg) > 3 else None
        if label:
            before = len(trace(segs[:i]))
            pts = trace(segs[:i + 1])[before - 1:]
            m = pts[len(pts) // 2]
            out.append((label, float(m[0]), float(m[1])))
    return out


def trace(segs, step=0.25):
    """Centreline points, heading and curvature radius (inf on straights)."""
    x, y, h = 0.0, 0.0, 0.0
    pts = [(x, y, h, math.inf)]
    for seg in segs:
        if seg[0] == 'S':
            n = max(1, int(round(seg[1] / step)))
            for _ in range(n):
                x += seg[1] / n * math.cos(h)
                y += seg[1] / n * math.sin(h)
                pts.append((x, y, h, math.inf))
        else:
            ang, r = math.radians(seg[1]), seg[2]
            n = max(2, int(round(abs(ang) * r / step)))
            d = ang / n
            for _ in range(n):
                cx, cy = x - math.sin(h) * r * np.sign(d), y + math.cos(h) * r * np.sign(d)
                h += d
                x = cx + math.sin(h) * r * np.sign(d)
                y = cy - math.cos(h) * r * np.sign(d)
                pts.append((x, y, h, r))
    return np.array(pts)


def solve():
    """Back-straight and west-side lengths that close the loop at the origin."""
    base = trace(layout(0.0, 0.0))[-1, :2]
    ex = trace(layout(1.0, 0.0))[-1, :2] - base
    ey = trace(layout(0.0, 1.0))[-1, :2] - base
    x1, x2 = np.linalg.solve(np.stack([ex, ey], 1), -base)
    return float(x1), float(x2)


def cone_row(c, side):
    """Cones along one row: offset WIDTH/2 to the left (+1) or right (-1)."""
    x = c[:, 0] - side * WIDTH / 2 * np.sin(c[:, 2])
    y = c[:, 1] + side * WIDTH / 2 * np.cos(c[:, 2])
    seg = np.hypot(np.diff(x), np.diff(y))
    out, run = [(x[0], y[0])], 0.0
    for i in range(1, len(c)):
        run += seg[i - 1]
        gap = CORNER_SPACING if math.isfinite(c[i, 3]) or math.isfinite(c[i - 1, 3]) \
            else STRAIGHT_SPACING
        if run >= gap - 1e-6:
            out.append((x[i], y[i]))
            run = 0.0
    if np.hypot(out[-1][0] - out[0][0], out[-1][1] - out[0][1]) < 1.5:
        out.pop()                                     # the loop's last cone = its first
    return np.array(out)


def check(c, cones):
    """Cones of one leg must stay clear of the other legs' driving line."""
    worst = math.inf
    s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(c[:, :2], axis=0).T))])
    for p in cones:
        d = np.hypot(c[:, 0] - p[0], c[:, 1] - p[1])
        own = s[np.argmin(d)]
        far = np.abs(s - own) > 20.0
        far &= np.abs(s - own) < s[-1] - 20.0
        if far.any():
            worst = min(worst, float(d[far].min()))
    return worst, float(s[-1])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                  'lidar_test_track.csv'))
    ap.add_argument('--plot', default='')
    a = ap.parse_args()

    x1, x2 = solve()
    if not (5 < x1 <= 80 and 5 < x2 <= 80):
        raise SystemExit(f'loop does not close with legal straights: {x1:.1f}, {x2:.1f} m')
    c = trace(layout(x1, x2))
    blue, yellow = cone_row(c, +1), cone_row(c, -1)   # anticlockwise: blue (left) inside
    clear, length = check(c, np.vstack([blue, yellow]))
    print(f'lap {length:.0f} m, back straight {x1:.1f} m, west side {x2:.1f} m, '
          f'{len(blue)} blue + {len(yellow)} yellow cones, closest cone to another leg\'s '
          f'centreline {clear:.2f} m')
    if clear < WIDTH / 2 + 1.0:
        raise SystemExit('legs too close: another leg\'s cones are near the driving line')

    # Start line 6 m ahead of the car: big orange cones each side, as at FS events.
    sx = START_BEHIND
    rows = [('car_start', -0.0, 0.0, 0.0)]
    for side in (+1, -1):
        for dx in (-0.3, 0.3):
            rows.append(('big_orange', sx + dx, side * (WIDTH / 2 + 0.3), 0.0))
    near_start = lambda p: abs(p[1]) < WIDTH and abs(p[0] - sx) < 1.5   # noqa: E731
    rows += [('blue', p[0], p[1], 0.0) for p in blue if not near_start(p)]
    rows += [('yellow', p[0], p[1], 0.0) for p in yellow if not near_start(p)]
    with open(a.out, 'w') as f:
        f.write('tag,x,y,direction,x_variance,y_variance,xy_covariance\n')
        for tag, x, y, d in rows:
            f.write(f'{tag},{x:.4f},{y:.4f},{d},0.01,0.01,0.0\n')
    print(f'written {a.out}')

    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(12, 9))
        ax.plot(c[:, 0], c[:, 1], 'k-', lw=0.5)
        ax.scatter(blue[:, 0], blue[:, 1], s=8, c='b')
        ax.scatter(yellow[:, 0], yellow[:, 1], s=8, c='gold')
        ax.scatter([sx] * 4, [WIDTH / 2 + .3] * 2 + [-WIDTH / 2 - .3] * 2, s=20, c='orange')
        ax.plot(0, 0, 'k>', ms=10)
        ax.set_aspect('equal')
        ax.grid(alpha=.3)
        ax.set_title(f'lidar_test_track: {length:.0f} m')
        fig.savefig(a.plot, dpi=80, bbox_inches='tight')


if __name__ == '__main__':
    main()
