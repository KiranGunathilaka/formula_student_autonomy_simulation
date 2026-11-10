"""Centreline planner on synthetic tracks (run: python3 -m pytest test/)."""
import numpy as np

from falcon_planning import centerline, pursuit


def _lane(y_left, y_right, x0=-2.0, x1=30.0, step=2.5):
    xs = np.arange(x0, x1, step)
    return (np.stack([xs, np.full_like(xs, y_left)], 1),
            np.stack([xs, np.full_like(xs, y_right)], 1))


def test_straight_follows_middle():
    blue, yellow = _lane(1.8, -1.8)
    path, _, _ = centerline.plan_centerline(blue, yellow)
    assert len(path) > 10
    assert np.allclose(path[0], (0, 0))
    assert np.max(np.abs(path[path[:, 0] > 2, 1])) < 0.1
    assert path[-1, 0] > 20


def test_ignores_parallel_lane_of_same_colours():
    # Our lane between y=+1.5 (blue) and y=-1.5 (yellow); another lane beside it whose
    # blue row is only 3 m away from ours (a serpentine).  The path must stay in ours.
    blue, yellow = _lane(1.5, -1.5)
    blue2, yellow2 = _lane(4.5, 7.5)       # neighbour: blue at 4.5, yellow at 7.5
    path, _, _ = centerline.plan_centerline(np.vstack([blue, blue2]), np.vstack([yellow, yellow2]))
    assert np.max(np.abs(path[:, 1])) < 0.3


def test_left_corner_turns_left():
    # 90 deg left turn, inner radius 3 m, outer 6.5 m, centred at (8, 4.75).
    th = np.linspace(-np.pi / 2, 0, 9)
    inner = np.stack([8 + 3.0 * np.cos(th), 4.75 + 3.0 * np.sin(th)], 1)
    outer = np.stack([8 + 6.5 * np.cos(th), 4.75 + 6.5 * np.sin(th)], 1)
    xs = np.arange(-2, 8, 2.5)
    blue = np.vstack([np.stack([xs, np.full_like(xs, 1.75)], 1), inner])
    yellow = np.vstack([np.stack([xs, np.full_like(xs, -1.75)], 1), outer])
    path, _, _ = centerline.plan_centerline(blue, yellow)
    r = np.hypot(path[:, 0] - 8, path[:, 1] - 4.75)
    on_arc = path[:, 0] > 9
    assert on_arc.any()
    assert np.all((r[on_arc] > 3.5) & (r[on_arc] < 6.0))   # between the cone rows


def test_one_side_only_offsets_by_half_track():
    blue, _ = _lane(1.5, -1.5)
    path, _, _ = centerline.plan_centerline(blue, np.zeros((0, 2)))
    mid = path[(path[:, 0] > 3) & (path[:, 0] < 15)]
    assert len(mid) and np.allclose(mid[:, 1], 1.5 - 1.5, atol=0.2)


def test_orange_line_on_sparse_cones_keeps_its_sides():
    # Worst legal cones: 5 m apart on a 3 m track, the yellows staggered 2.5 m from the
    # blues.  The orange line replaces the blue at x = 10, so that orange's nearest cone
    # is the yellow across the track, not a blue.  Each orange must still keep its side.
    blue = np.array([(x, 1.5) for x in (0.0, 5.0, 15.0, 20.0, 25.0)])
    yellow = np.array([(x, -1.5) for x in (2.5, 7.5, 12.5, 17.5, 22.5)])
    orange = np.array([(10.0, 1.5), (10.4, 1.5), (10.0, -1.5), (10.4, -1.5)])
    sides = centerline._orange_sides(orange, blue, yellow, centerline.DEFAULTS)
    assert list(sides) == [1, 1, -1, -1]
    path, _, _ = centerline.plan_centerline(blue, yellow, orange=orange)
    assert path[-1, 0] > 20 and np.max(np.abs(path[:, 1])) < 0.3


def test_lookahead_on_path_and_steering_sign():
    path = np.stack([np.linspace(0, 10, 21), np.linspace(0, 2, 21)], 1)
    lx, ly = pursuit.lookahead_point(path, 3.0)
    assert abs(np.hypot(lx, ly) - 3.0) < 1e-6
    assert pursuit.steering_angle((lx, ly), 1.53, -0.765) > 0   # path bends left
