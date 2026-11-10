"""
Centerline geometry for the path planner (no ROS imports, so it can be tested offline).

All points are in the vehicle frame (x forward, y left), the car at the origin.

Why this replaced the old midpoint planner
------------------------------------------
The old planner paired every yellow cone with its nearest blue cone over the whole
merged map, one-to-one and greedily, then ordered the midpoints nearest-neighbour from
the car.  Three things went wrong:

  * Corners have more cones on the outside than on the inside, so 1-to-1 pairing
    leaves diagonal pairs whose midpoints sit off the centreline, and once the map
    holds the whole track a cone can be paired with one from another part of the
    track.
  * Nearest-neighbour ordering jumps between those midpoints, so the path zig-zags.
  * Pairing all map cones is O(n^2 log n) every cycle: about 0.5 s per cycle on the
    984-cone hairpin track, so the 10 Hz planner fell behind and pure pursuit timed out.

The planner now:

  1. Keeps only cones in a window around the car (``window_radius_m``).
  2. Triangulates them (Delaunay).  Only an edge joining a left cone to a right cone
     can be crossed by the centreline; its midpoint is a waypoint.  Two same-colour
     rows belonging to different parts of the track only form same-colour edges, so
     they are a wall the path cannot pass, however close they are.
  3. Walks from the car through adjacent triangles, edge midpoint to edge midpoint,
     always taking the crossable edge with the smallest turn.
  4. If that walk is short (one side not seen: a hairpin's inside, a missed cone,
     the first metres of a run) it also chains each boundary from the car forward
     and pairs each cone with the other side or with a virtual cone
     ``track_width_m`` across, and keeps whichever path is longer.
  5. Smooths and resamples at ``resample_step_m``.
"""

import math
import warnings

import numpy as np

try:
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        from scipy.spatial import Delaunay as _ScipyDelaunay
except Exception:  # scipy missing or built against another numpy
    _ScipyDelaunay = None


DEFAULTS = {
    'window_radius_m': 30.0,       # ignore cones further than this from the car
    'window_behind_m': 3.0,        # ...or further than this behind the car
    'max_cone_spacing_m': 6.5,     # rules cap cone spacing at 5 m; leave margin
    'max_turn_deg': 80.0,          # max change of boundary direction per cone
    'track_width_m': 3.0,          # virtual cone offset (FS rules: >= 3 m)
    'min_track_width_m': 1.8,      # partner closer than this is rejected
    # Partner further than this is rejected.  Every EUFS track but the random one
    # keeps left-right gaps under 5.9 m; above ~6.5 m an edge can span two parts of
    # the track (e.g. the waist of `peanut`).
    'max_track_width_m': 6.0,
    'max_path_length_m': 25.0,     # stop chaining after this boundary length
    'merge_radius_m': 0.8,         # midpoints closer than this are averaged
    'resample_step_m': 0.5,        # output spacing
    'smoothing_passes': 2,         # moving-average passes over the midpoints
    'max_walk_turn_deg': 75.0,     # Delaunay walk: max heading change per waypoint
    # Shorter than this (in practice: only one side in view) the boundary chains
    # with virtual cones are tried too.  Kept small because a one-sided chain can
    # cut through the inside of the track where the other side is out of view.
    'min_delaunay_path_m': 3.0,
    'min_cone_clearance_m': 1.0,   # a chained path is cut where it passes this close to a cone
}


def chain_boundary(points, side, p):
    """Order one side's cones from the car forward.

    points: (N, 2) array of cones of one colour, vehicle frame.
    side:   +1 for the left (blue) boundary, -1 for the right (yellow) boundary.
    Returns an (M, 2) array, M <= N, in driving order.
    """
    if len(points) == 0:
        return np.zeros((0, 2))
    max_sp = p['max_cone_spacing_m']
    cos_turn = math.cos(math.radians(p['max_turn_deg']))
    half_w = 0.5 * p['track_width_m']

    # Virtual start: where this boundary would be beside the car.
    prev = np.array([0.0, side * half_w])
    direction = np.array([1.0, 0.0])
    used = np.zeros(len(points), dtype=bool)
    chain = []
    length = 0.0

    while length < p['max_path_length_m']:
        vec = points - prev
        dist = np.hypot(vec[:, 0], vec[:, 1])
        with np.errstate(invalid='ignore', divide='ignore'):
            cos_a = (vec @ direction) / dist
        ok = (~used) & (dist > 0.2) & (dist <= max_sp) & (cos_a >= cos_turn)
        if not chain:
            # First cone: beside or ahead of the car, on its own side of the car.
            ok &= (points[:, 0] > -p['window_behind_m']) & (side * points[:, 1] > -0.5)
        if not ok.any():
            break
        # Prefer near cones that keep the direction; angle cost grows quickly.
        ang = np.arccos(np.clip(cos_a, -1.0, 1.0))
        cost = np.where(ok, dist * (1.0 + 2.0 * ang * ang), np.inf)
        i = int(np.argmin(cost))
        step = points[i] - prev
        step_len = float(np.hypot(*step))
        used[i] = True
        chain.append(points[i])
        if len(chain) > 1:
            length += step_len
        new_dir = step / step_len
        # Light smoothing of the direction keeps one noisy cone from steering the chain.
        direction = new_dir if len(chain) == 1 else _unit(0.3 * direction + 0.7 * new_dir)
        prev = points[i]

    return np.array(chain) if chain else np.zeros((0, 2))


def _unit(v):
    n = float(np.hypot(*v))
    return v / n if n > 1e-9 else np.array([1.0, 0.0])


def _chain_normals(chain, side):
    """Unit normals pointing from this boundary into the track."""
    n = len(chain)
    if n == 0:
        return np.zeros((0, 2))
    dirs = np.zeros((n, 2))
    for i in range(n):
        a = chain[max(i - 1, 0)]
        b = chain[min(i + 1, n - 1)]
        if n == 1:
            d = np.array([1.0, 0.0])
        else:
            d = _unit(b - a)
        dirs[i] = d
    # Left boundary: inside is to its right (-90 deg); right boundary: to its left.
    if side > 0:
        return np.stack([dirs[:, 1], -dirs[:, 0]], axis=1)
    return np.stack([-dirs[:, 1], dirs[:, 0]], axis=1)


def _closest_on_polyline(q, poly):
    """Closest point to q on polyline poly (M, 2); returns (point, distance, s)."""
    if len(poly) == 1:
        d = float(np.hypot(*(poly[0] - q)))
        return poly[0], d, 0.0
    a = poly[:-1]
    b = poly[1:]
    ab = b - a
    seg_len2 = np.einsum('ij,ij->i', ab, ab)
    seg_len2[seg_len2 < 1e-12] = 1e-12
    t = np.clip(np.einsum('ij,ij->i', q - a, ab) / seg_len2, 0.0, 1.0)
    proj = a + ab * t[:, None]
    d = np.hypot(proj[:, 0] - q[0], proj[:, 1] - q[1])
    k = int(np.argmin(d))
    seg_s = np.concatenate([[0.0], np.cumsum(np.sqrt(seg_len2))])
    s = seg_s[k] + t[k] * math.sqrt(seg_len2[k])
    return proj[k], float(d[k]), float(s)


def _midpoints(chain, other, side, p):
    """One midpoint per cone of `chain`, partnered with `other` or a virtual cone."""
    mids = []
    normals = _chain_normals(chain, side)
    for c, nrm in zip(chain, normals):
        partner = None
        if len(other):
            q, d, _ = _closest_on_polyline(c, other)
            across = float((q - c) @ nrm)
            if (p['min_track_width_m'] <= d <= p['max_track_width_m']
                    and across > 0.5 * d):
                partner = q
        if partner is None:
            partner = c + nrm * p['track_width_m']
        mids.append(0.5 * (c + partner))
    return np.array(mids) if mids else np.zeros((0, 2))


def _order_and_merge(mids_a, mids_b, p):
    ref = mids_a if len(mids_a) >= len(mids_b) else mids_b
    allm = np.vstack([m for m in (mids_a, mids_b) if len(m)])
    if len(ref) == 0:
        return np.zeros((0, 2))
    ref = np.vstack([[0.0, 0.0], ref])
    keyed = []
    for m in allm:
        _, d, s = _closest_on_polyline(m, ref)
        if d < p['track_width_m']:           # drop midpoints far off the reference
            keyed.append((s, m))
    keyed.sort(key=lambda e: e[0])
    merged = []
    for _, m in keyed:
        if merged and np.hypot(*(merged[-1][0] / merged[-1][1] - m)) < p['merge_radius_m']:
            merged[-1][0] += m
            merged[-1][1] += 1
        else:
            merged.append([m.copy(), 1])
    pts = np.array([acc / cnt for acc, cnt in merged])
    # Never route behind the car.
    return pts[pts[:, 0] > -0.5] if len(pts) else pts


def _smooth(pts, passes):
    if len(pts) < 3:
        return pts
    out = pts.copy()
    for _ in range(passes):
        nxt = out.copy()
        nxt[1:-1] = 0.25 * out[:-2] + 0.5 * out[1:-1] + 0.25 * out[2:]
        out = nxt
    return out


def resample(pts, step):
    """Resample a polyline at a fixed arc-length step (keeps both ends)."""
    if len(pts) < 2:
        return pts
    seg = np.hypot(*np.diff(pts, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] < 1e-6:
        return pts[:1]
    targets = np.arange(0.0, s[-1], step)
    targets = np.append(targets, s[-1])
    x = np.interp(targets, s, pts[:, 0])
    y = np.interp(targets, s, pts[:, 1])
    return np.stack([x, y], axis=1)


def _triangulate(pts):
    """Delaunay triangles as an (M, 3) index array."""
    if len(pts) < 3:
        return np.zeros((0, 3), dtype=int)
    if _ScipyDelaunay is not None:
        try:
            return np.asarray(_ScipyDelaunay(pts).simplices, dtype=int)
        except Exception:  # degenerate input (all cones in a line)
            return np.zeros((0, 3), dtype=int)
    return _bowyer_watson(pts)


def _bowyer_watson(pts):
    """Plain O(n^2) Bowyer-Watson, used only when scipy is unavailable."""
    n = len(pts)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    span = max(float(np.max(hi - lo)), 1.0) * 20.0
    mid = 0.5 * (lo + hi)
    verts = np.vstack([pts, [mid + [-span, -span], mid + [span, -span], mid + [0.0, span]]])
    tris = [(n, n + 1, n + 2)]
    circ = [_circumcircle(verts, tris[0])]
    for i in range(n):
        p = verts[i]
        c = np.array([cc[0] for cc in circ])
        r2 = np.array([cc[1] for cc in circ])
        bad = np.nonzero(np.sum((c - p) ** 2, axis=1) < r2)[0]
        edge_count = {}
        for b in bad:
            a0, a1, a2 = tris[b]
            for e in ((a0, a1), (a1, a2), (a2, a0)):
                k = (min(e), max(e))
                edge_count[k] = edge_count.get(k, 0) + 1
        keep = np.ones(len(tris), dtype=bool)
        keep[bad] = False
        tris = [t for t, k in zip(tris, keep) if k]
        circ = [cc for cc, k in zip(circ, keep) if k]
        for (e0, e1), cnt in edge_count.items():
            if cnt == 1:
                t = (e0, e1, i)
                cc = _circumcircle(verts, t)
                if cc is not None:
                    tris.append(t)
                    circ.append(cc)
    out = [t for t in tris if max(t) < n]
    return np.array(out, dtype=int) if out else np.zeros((0, 3), dtype=int)


def _circumcircle(verts, t):
    (ax, ay), (bx, by), (cx, cy) = verts[t[0]], verts[t[1]], verts[t[2]]
    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        return (np.array([0.0, 0.0]), -1.0)  # degenerate: never "bad"
    ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay)
          + (cx * cx + cy * cy) * (ay - by)) / d
    uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx)
          + (cx * cx + cy * cy) * (bx - ax)) / d
    centre = np.array([ux, uy])
    return (centre, float(np.sum((centre - verts[t[0]]) ** 2)))


def _orange_sides(orange, left, right, p):
    """Give each orange cone a side (+1 left / -1 right) from the blue/yellow cones."""
    if len(orange) == 0:
        return np.zeros(0)
    ref = [a for a in (left, right) if len(a)]
    if not ref:
        return np.where(orange[:, 1] >= 0, 1.0, -1.0)
    d_l = (np.min(np.hypot(*(orange[:, None, :] - left[None, :, :]).transpose(2, 0, 1)), axis=1)
           if len(left) else np.full(len(orange), np.inf))
    d_r = (np.min(np.hypot(*(orange[:, None, :] - right[None, :, :]).transpose(2, 0, 1)), axis=1)
           if len(right) else np.full(len(orange), np.inf))
    sides = np.where(d_l <= d_r, 1.0, -1.0)
    if not (len(left) and len(right)):
        return sides
    # With cones 5 m apart on a 3 m track the cone straight across can be nearer than the
    # next one on the same side, so the nearest cone alone can put an orange on the wrong
    # side.  Start and finish lines have an orange on each edge: split each orange and its
    # partner across the track whichever way is nearer overall, so the along-track offset
    # counts the same for both.
    d_o = np.hypot(*(orange[:, None, :] - orange[None, :, :]).transpose(2, 0, 1))
    across = (d_o >= p['min_track_width_m']) & (d_o <= p['max_track_width_m'])
    for i in range(len(orange)):
        if across[i].any():
            j = int(np.argmin(np.where(across[i], d_o[i], np.inf)))
            sides[i] = 1.0 if d_l[i] + d_r[j] <= d_r[i] + d_l[j] else -1.0
    return sides


def delaunay_walk(pts, side, p, prev_path=None):
    """Walk the corridor of left-right Delaunay edges from the car forward.

    pts: (N, 2) cones, side: (N,) +1 left / -1 right.  Returns (K, 2) midpoints in
    driving order (without the car origin), possibly empty.

    prev_path: the last published path moved into the current vehicle frame, or
    None.  When the car has left the corridor (ran wide in a hairpin it cannot make)
    the corridor nearest the car can belong to another part of the track; the walk
    is then also started from the edge nearest the previous path, and the walk that
    stays closer to the previous path wins.
    """
    tris = _triangulate(pts)
    if len(tris) == 0:
        return np.zeros((0, 2))
    lo_w, hi_w = p['min_track_width_m'], p['max_track_width_m']

    def crossable(a, b):
        if side[a] == side[b]:
            return False
        w = float(np.hypot(*(pts[a] - pts[b])))
        return lo_w <= w <= hi_w

    edge_tris = {}
    tri_cross = []
    for ti, (a, b, c) in enumerate(tris):
        ce = []
        for e in ((a, b), (b, c), (c, a)):
            k = (min(e), max(e))
            edge_tris.setdefault(k, []).append(ti)
            if crossable(*k):
                ce.append(k)
        tri_cross.append(ce)

    def mid(k):
        return 0.5 * (pts[k[0]] + pts[k[1]])

    # Which triangle holds the car?
    a, b, c = pts[tris[:, 0]], pts[tris[:, 1]], pts[tris[:, 2]]
    v0, v1 = b - a, c - a
    v2 = -a
    den = v0[:, 0] * v1[:, 1] - v1[:, 0] * v0[:, 1]
    with np.errstate(divide='ignore', invalid='ignore'):
        u = (v2[:, 0] * v1[:, 1] - v1[:, 0] * v2[:, 1]) / den
        w = (v0[:, 0] * v2[:, 1] - v2[:, 0] * v0[:, 1]) / den
    inside = np.nonzero((u >= 0) & (w >= 0) & (u + w <= 1) & (np.abs(den) > 1e-12))[0]

    def leave_away_from(edge, origin):
        """The triangle of `edge` to walk *into*: the one further from `origin`."""
        cands = edge_tris[edge]
        if len(cands) < 2:
            return None
        d_m = _unit(mid(edge) - origin)
        cent = [pts[tris[t]].mean(axis=0) @ d_m for t in cands]
        return cands[int(np.argmin(cent))]

    starts = []
    car_tri = int(inside[0]) if len(inside) else None
    if car_tri is not None and tri_cross[car_tri]:
        ahead = [k for k in tri_cross[car_tri] if mid(k)[0] > -0.5]
        if ahead:
            starts.append((max(ahead, key=lambda k: mid(k)[0]), car_tri))
    if not starts:
        best, best_d = None, np.inf
        for k in edge_tris:
            if not crossable(*k):
                continue
            m = mid(k)
            d = float(np.hypot(*m))
            if 0.0 < m[0] and d < 8.0 and abs(math.atan2(m[1], m[0])) < math.radians(60) \
                    and d < best_d:
                best, best_d = k, d
        if best is not None:
            starts.append((best, leave_away_from(best, np.zeros(2))))
    in_corridor = car_tri is not None and bool(tri_cross[car_tri])
    has_prev = prev_path is not None and len(prev_path) >= 2 and not in_corridor
    if has_prev:
        # Edge whose midpoint lies on the previous path, a little ahead of the car.
        best, best_d = None, np.inf
        for k in edge_tris:
            if not crossable(*k):
                continue
            m = mid(k)
            if m[0] < 0.0 or np.hypot(*m) > 8.0:
                continue
            _, d, _ = _closest_on_polyline(m, prev_path)
            if d < best_d:
                best, best_d = k, d
        if best is not None and best_d < 1.5 and all(best != s for s, _ in starts):
            starts.append((best, leave_away_from(best, np.zeros(2))))
    if not starts:
        return np.zeros((0, 2))

    max_turn = math.radians(p['max_walk_turn_deg'])
    walks = [_walk(s, came, mid, edge_tris, tri_cross, max_turn, p) for s, came in starts]
    if len(walks) == 1 or not has_prev:
        return walks[0]

    def deviation(w):
        pts_w = resample(np.vstack([[0.0, 0.0], w]), 0.5)
        s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts_w, axis=0).T))])
        near = pts_w[(s > 1.0) & (s < 8.0)]
        if len(near) == 0:
            return np.inf
        return float(np.mean([_closest_on_polyline(q, prev_path)[1] for q in near]))

    return min(walks, key=deviation)


def _walk(start_edge, came_from, mid, edge_tris, tri_cross, max_turn, p):
    """Follow crossable edges from `start_edge`, leaving `came_from` behind."""
    path = [mid(start_edge)]
    prev = np.zeros(2)
    visited = {start_edge}
    cur, cur_from = start_edge, came_from
    length = float(np.hypot(*path[0]))
    while length < p['max_path_length_m']:
        nxt_tris = [t for t in edge_tris[cur] if t != cur_from]
        if not nxt_tris:
            break
        t = nxt_tris[0]
        exits = [k for k in tri_cross[t] if k != cur and k not in visited]
        if not exits:
            break
        here = path[-1]
        heading = here - prev
        hn = float(np.hypot(*heading))
        best, best_turn = None, np.inf
        for k in exits:
            step = mid(k) - here
            sn = float(np.hypot(*step))
            if sn < 1e-6 or hn < 1e-6:
                turn = 0.0
            else:
                turn = math.acos(max(-1.0, min(1.0, float(step @ heading) / (sn * hn))))
            if turn < best_turn:
                best, best_turn = k, turn
        if best_turn > max_turn:
            break
        m = mid(best)
        step_len = float(np.hypot(*(m - here)))
        # Keep the heading reference at least ~1 m back so tight fans don't read as turns.
        if step_len > 1.0 or hn < 1.0:
            prev = here
        path.append(m)
        length += step_len
        visited.add(best)
        cur, cur_from = best, t
    return np.array(path)


def _path_length(pts):
    if len(pts) < 2:
        return 0.0
    return float(np.sum(np.hypot(*np.diff(pts, axis=0).T)))


def plan_centerline(blue, yellow, params=None, orange=None, prev_path=None):
    """Return (path (K, 2), left_chain, right_chain) in the vehicle frame.

    blue / yellow / orange: (N, 2) arrays (any may be empty).  Orange cones mark the
    start/finish area and sit in either edge; each takes the side of its nearest
    blue/yellow cone.  The path starts at the car (0, 0).  It is empty when no
    boundary was found.  The chains are returned for visualisation (empty when the
    Delaunay walk alone was long enough).  prev_path: see delaunay_walk.
    """
    p = dict(DEFAULTS)
    if params:
        p.update(params)
    blue = _window(np.asarray(blue, dtype=float).reshape(-1, 2), p)
    yellow = _window(np.asarray(yellow, dtype=float).reshape(-1, 2), p)
    orange = _window(np.asarray(orange if orange is not None else [], dtype=float)
                     .reshape(-1, 2), p)

    o_side = _orange_sides(orange, blue, yellow, p)
    left_pts = np.vstack([blue, orange[o_side > 0]]) if len(orange) else blue
    right_pts = np.vstack([yellow, orange[o_side < 0]]) if len(orange) else yellow

    pts = np.vstack([left_pts, right_pts])
    side = np.concatenate([np.ones(len(left_pts)), -np.ones(len(right_pts))])
    pts, side = _dedupe(pts, side)
    if prev_path is not None:
        prev_path = np.asarray(prev_path, dtype=float).reshape(-1, 2)
    mids = delaunay_walk(pts, side, p, prev_path) if len(pts) >= 3 else np.zeros((0, 2))
    walk = np.vstack([[0.0, 0.0], mids]) if len(mids) else np.zeros((0, 2))

    left = right = np.zeros((0, 2))
    if _path_length(walk) < p['min_delaunay_path_m']:
        left = chain_boundary(left_pts, +1, p)
        right = chain_boundary(right_pts, -1, p)
        if len(left) or len(right):
            mids_l = _midpoints(left, right, +1, p)
            mids_r = _midpoints(right, left, -1, p)
            cm = _order_and_merge(mids_l, mids_r, p)
            chained = np.vstack([[0.0, 0.0], cm]) if len(cm) else np.zeros((0, 2))
            chained = _cut_at_clearance(chained, pts, p['min_cone_clearance_m'])
            if _path_length(chained) > _path_length(walk):
                walk = chained

    if len(walk) < 2:
        return np.zeros((0, 2)), left, right
    path = _smooth(walk, p['smoothing_passes'])
    path[0] = (0.0, 0.0)
    return resample(path, p['resample_step_m']), left, right


def _cut_at_clearance(path, cones, clearance):
    """Truncate `path` before the first point (past the first metre) too close to a cone."""
    if len(path) < 2 or len(cones) == 0:
        return path
    dense = resample(path, 0.25)
    d = np.min(np.hypot(dense[:, None, 0] - cones[None, :, 0],
                        dense[:, None, 1] - cones[None, :, 1]), axis=1)
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(dense, axis=0).T))])
    bad = np.nonzero((d < clearance) & (s > 1.0))[0]
    return dense[:bad[0]] if len(bad) else dense


def _dedupe(pts, side, radius=0.25):
    """Drop cones within `radius` of an earlier one (map + live copies of one cone)."""
    if len(pts) < 2:
        return pts, side
    keep = np.ones(len(pts), dtype=bool)
    for i in range(len(pts)):
        if not keep[i]:
            continue
        d = np.hypot(pts[i + 1:, 0] - pts[i, 0], pts[i + 1:, 1] - pts[i, 1])
        keep[i + 1:][d < radius] = False
    return pts[keep], side[keep]


def _window(pts, p):
    if len(pts) == 0:
        return pts.reshape(0, 2)
    r2 = p['window_radius_m'] ** 2
    keep = (pts[:, 0] ** 2 + pts[:, 1] ** 2 < r2) & (pts[:, 0] > -p['window_behind_m'])
    return pts[keep]


def path_curvature(path):
    """Unsigned curvature at each interior point (Menger curvature), ends copied."""
    n = len(path)
    k = np.zeros(n)
    if n < 3:
        return k
    a, b, c = path[:-2], path[1:-1], path[2:]
    ab = np.hypot(*(b - a).T)
    bc = np.hypot(*(c - b).T)
    ca = np.hypot(*(a - c).T)
    cross = np.abs((b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1])
                   - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0]))
    denom = ab * bc * ca
    k[1:-1] = np.where(denom > 1e-9, 2.0 * cross / np.maximum(denom, 1e-9), 0.0)
    k[0] = k[1]
    k[-1] = k[-2]
    return k
