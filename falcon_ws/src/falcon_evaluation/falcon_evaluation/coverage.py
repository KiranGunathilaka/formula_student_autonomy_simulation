"""
Cone coverage: which track cones did the lidar actually put points on?

Pure numpy (no ROS) so it can be unit-tested and reused offline.

Every scan, each ground-truth cone is classed as

  in view    inside the lidar's horizontal and vertical field of view and range
             (geometry only: it may still be hidden behind another cone or the car);
  hit        at least one lidar return on it;
  detected   at least `min_points` returns from at least `min_rings` different
             channels: enough for a clustering detector to find it.

and over the whole run a cone is

  detected       detected in at least one scan;
  hit only       had returns but never enough;
  missed         was in view but never had a return (occluded, or fell between rings);
  never in view  never inside the field of view at all.

Two ways to get the returns on each cone:

  add_scan_raycast  casts the lidar profile's exact rays (every channel x column)
                    from the true pose against the cones (frustums), the flat ground
                    and the car's body box.  Exact and fast, and independent of how
                    Gazebo renders the lidar.  This is the default.
  add_scan_points   counts the points of a real cloud (e.g. Gazebo's /lidar/points)
                    within `catch_radius_m` of each cone's axis, between
                    `ground_clearance_m` and the cone's height (+5 cm).

The ground is z = 0 of the vehicle frame (flat simulated track).  With `body_box`
(x_min, x_max, y_half in the vehicle frame) it also records cones that ended up
inside the car's footprint: the lap behind the numbers was not clean.
"""

import math

import numpy as np

COLOR_NAMES = {0: 'unknown', 1: 'blue', 2: 'yellow', 3: 'orange', 4: 'big_orange'}

# Cone bodies as frustums: (base radius, top radius, height) in metres.
CONE_SHAPES = {
    # FS cones: small 228 x 228 x 325 mm, big 285 x 285 x 505 mm.  The small frustum is
    # the one lidar_coverage_and_hairpins.html uses; the big one is scaled alike.
    'fs': {'small': (0.114, 0.025, 0.325), 'big': (0.1425, 0.03, 0.505)},
    # The eufs_tracks meshes (measured from cone_blue.dae / cone_big.dae): slimmer
    # and a little shorter.  Use this to compare with Gazebo's own lidar.
    'eufs': {'small': (0.080, 0.012, 0.304), 'big': (0.100, 0.030, 0.531)},
}

RANGE_BINS = (0, 5, 10, 15, 20, 25, 30, 40, 50, 75, 100, 150, 200, 250)


class CoverageAccumulator:
    def __init__(self, cones_map, channels_deg, hfov_deg, hcentre_deg=0.0,
                 max_range_m=100.0, min_points=3, min_rings=2,
                 catch_radius_m=0.25, ground_clearance_m=0.03, body_box=None,
                 cone_shape='fs', h_samples=1800, min_range_m=0.3, occluder_box=None,
                 record=False):
        """cones_map: (N, 3) x, y, colour code in the map frame.
        h_samples, min_range_m: columns per frame and blind range (raycast mode).
        occluder_box: (x_min, x_max, y_half, z_max) of the bodywork in the vehicle
        frame; rays that enter it stop (raycast mode, ignored if the lidar is inside).
        record: keep every scan's points, channels and range per cone (frames()), so
        statistics can be computed from the whole run afterwards (lidar_stats)."""
        self.cones = np.asarray(cones_map, dtype=float).reshape(-1, 3)
        shapes = CONE_SHAPES[cone_shape]
        big = self.cones[:, 2] == 4
        self.r0 = np.where(big, shapes['big'][0], shapes['small'][0])
        self.r1 = np.where(big, shapes['big'][1], shapes['small'][1])
        self.height = np.where(big, shapes['big'][2], shapes['small'][2])
        self.channels = np.sort(np.asarray(channels_deg, dtype=float))
        self.h_samples = int(h_samples)
        self.min_range = min_range_m
        self.occluder = occluder_box
        self.hfov = math.radians(hfov_deg)
        self.hcentre = math.radians(hcentre_deg)
        self.max_range = max_range_m
        self.min_points = min_points
        self.min_rings = min_rings
        self.r = catch_radius_m
        self.clear = ground_clearance_m
        self.body_box = body_box
        # half the gap to the next channel, so an elevation maps to its channel
        gaps = np.diff(self.channels)
        self._ch_margin = (gaps.min() / 2) if len(gaps) else 0.5

        n = len(self.cones)
        self.scans = 0
        self.in_view = np.zeros(n, dtype=int)
        self.hit = np.zeros(n, dtype=int)
        self.detected = np.zeros(n, dtype=int)
        self.max_points = np.zeros(n, dtype=int)
        self.max_rings = np.zeros(n, dtype=int)
        self.first_detect_range = np.full(n, np.nan)   # furthest range when detected
        self.closest_in_view = np.full(n, np.nan)
        self.touched = np.zeros(n, dtype=bool)
        nb = len(RANGE_BINS) - 1
        self.bin_in_view = np.zeros(nb, dtype=int)
        self.bin_detected = np.zeros(nb, dtype=int)
        self.bin_points = np.zeros(nb, dtype=float)
        self.record = record
        self._frames = []

    # ------------------------------------------------------------------ #
    def _geometry(self, R_sb, t_sb, map_to_base):
        """Cones in the vehicle frame, their range from the lidar, and the view mask."""
        c, s, tx, ty = map_to_base
        cx = c * self.cones[:, 0] - s * self.cones[:, 1] + tx
        cy = s * self.cones[:, 0] + c * self.cones[:, 1] + ty
        if self.body_box is not None:
            x0, x1, yh = self.body_box[:3]
            self.touched |= (cx > x0) & (cx < x1) & (np.abs(cy) < yh)

        # Geometric view: cone axis at mid-height in the lidar frame.
        rng = np.hypot(cx - t_sb[0], cy - t_sb[1])
        base_pts = np.stack([cx, cy, np.zeros_like(cx)], axis=1)
        top_pts = np.stack([cx, cy, self.height], axis=1)
        bs = (base_pts - t_sb) @ R_sb      # base -> sensor (R^T applied row-wise)
        ts = (top_pts - t_sb) @ R_sb
        mid = 0.5 * (bs + ts)
        az = np.arctan2(mid[:, 1], mid[:, 0]) - self.hcentre
        az = (az + np.pi) % (2 * np.pi) - np.pi
        el_lo = np.degrees(np.arctan2(bs[:, 2], np.hypot(bs[:, 0], bs[:, 1])))
        el_hi = np.degrees(np.arctan2(ts[:, 2], np.hypot(ts[:, 0], ts[:, 1])))
        in_h = np.abs(az) <= self.hfov / 2 + 1e-9 if self.hfov < 2 * np.pi - 1e-6 \
            else np.ones(len(az), dtype=bool)
        in_v = (el_hi >= self.channels[0] - self._ch_margin) & \
               (el_lo <= self.channels[-1] + self._ch_margin)
        view = in_h & in_v & (rng <= self.max_range)
        return cx, cy, rng, view, bs, ts

    def add_scan_points(self, pts_sensor, R_sb, t_sb, map_to_base):
        """Count the points of a cloud on each cone.

        pts_sensor: (M, 3) in the lidar frame.  R_sb, t_sb: lidar -> base_footprint.
        map_to_base: (cos, sin, tx, ty), the 2D transform taking map points to base."""
        cx, cy, rng, view, _, _ = self._geometry(R_sb, t_sb, map_to_base)
        P = np.asarray(pts_sensor, dtype=float).reshape(-1, 3)
        Pb = P @ R_sb.T + t_sb

        # Candidate points: above the ground, below the tallest cone.
        z = Pb[:, 2]
        cand = (z > self.clear) & (z < self.height.max() + 0.05)
        Pc, Ps = Pb[cand], P[cand]
        el = np.degrees(np.arctan2(Ps[:, 2], np.hypot(Ps[:, 0], Ps[:, 1])))
        ring = np.clip(np.searchsorted(self.channels, el), 0, len(self.channels) - 1)
        lower = np.clip(ring - 1, 0, len(self.channels) - 1)
        ring = np.where(np.abs(self.channels[lower] - el) < np.abs(self.channels[ring] - el),
                        lower, ring)

        order = np.argsort(Pc[:, 0])
        xs = Pc[order, 0]
        npts = np.zeros(len(cx), dtype=int)
        nring = np.zeros(len(cx), dtype=int)
        near = np.nonzero(rng <= self.max_range + 1.0)[0]
        for i in near:
            lo = np.searchsorted(xs, cx[i] - self.r)
            hi = np.searchsorted(xs, cx[i] + self.r)
            if hi <= lo:
                continue
            idx = order[lo:hi]
            d = np.hypot(Pc[idx, 0] - cx[i], Pc[idx, 1] - cy[i])
            m = (d <= self.r) & (Pc[idx, 2] <= self.height[i] + 0.05)
            k = int(m.sum())
            if k:
                npts[i] = k
                nring[i] = len(np.unique(ring[idx[m]]))
        return self._accumulate(npts, nring, view, rng)

    # Backwards-compatible name.
    add_scan = add_scan_points

    def add_scan_raycast(self, R_sb, t_sb, map_to_base):
        """Cast the profile's rays at every cone in view; nearest hit per ray wins."""
        cx, cy, rng, view, bs, ts = self._geometry(R_sb, t_sb, map_to_base)
        npts, nring = self._raycast(R_sb, np.asarray(t_sb, dtype=float), cx, cy, view, bs, ts)
        return self._accumulate(npts, nring, view, rng)

    def _columns(self):
        """First column azimuth and column step, as the Gazebo sensor lays them out."""
        n_h = self.h_samples
        if self.hfov >= 2 * np.pi - 1e-6:      # -pi .. pi - step
            return -np.pi, 2 * np.pi / n_h, True
        return self.hcentre - self.hfov / 2, self.hfov / max(n_h - 1, 1), False  # ends included

    def _occluder_for(self, O):
        occ = self.occluder
        if occ is not None and (occ[0] < O[0] < occ[1] and abs(O[1]) < occ[2] and O[2] < occ[3]):
            return None   # lidar inside the box: the box cannot be its occluder
        return occ

    def _raycast(self, R_sb, O, cx, cy, view, bs, ts):
        el = np.radians(self.channels)
        n_h = self.h_samples
        az0, daz, full = self._columns()
        occ = self._occluder_for(O)

        ray_ids, ts_hit, cone_ids = [], [], []
        for i in np.nonzero(view)[0]:
            # Angular window of the cone in the lidar frame.
            d_h = max(np.hypot(bs[i, 0], bs[i, 1]), 1e-3)
            hw = math.atan2(self.r0[i] + 0.01, d_h)
            a_b = math.atan2(bs[i, 1], bs[i, 0])
            a_t = math.atan2(ts[i, 1], ts[i, 0])
            a_mid = a_b + ((a_t - a_b + np.pi) % (2 * np.pi) - np.pi) / 2
            spread = abs((a_t - a_b + np.pi) % (2 * np.pi) - np.pi) / 2 + hw
            e_b = math.atan2(bs[i, 2], d_h)
            e_t = math.atan2(ts[i, 2], max(np.hypot(ts[i, 0], ts[i, 1]), 1e-3))
            e_lo, e_hi = min(e_b, e_t) - hw, max(e_b, e_t) + hw
            ch = np.arange(np.searchsorted(el, e_lo), np.searchsorted(el, e_hi, 'right'))
            if len(ch) == 0:
                continue
            k_lo = math.ceil((a_mid - spread - az0) / daz)
            k_hi = math.floor((a_mid + spread - az0) / daz)
            cols = np.arange(k_lo, k_hi + 1)
            if full:
                cols = np.unique(cols % n_h)
            else:
                cols = cols[(cols >= 0) & (cols < n_h)]
            if len(cols) == 0:
                continue
            E, A = np.meshgrid(el[ch], az0 + cols * daz, indexing='ij')
            dirs = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)],
                            axis=-1).reshape(-1, 3) @ R_sb.T
            ids = (ch[:, None] * n_h + cols[None, :]).reshape(-1)
            t = _ray_frustum(O, dirs, cx[i], cy[i], self.r0[i], self.r1[i], self.height[i])
            ok = (t >= self.min_range) & (t <= self.max_range)
            if not ok.any():
                continue
            dirs, ids, t = dirs[ok], ids[ok], t[ok]
            # Ground in front of the cone?
            with np.errstate(divide='ignore', invalid='ignore'):
                t_g = np.where(dirs[:, 2] < 0, -O[2] / dirs[:, 2], np.inf)
            keep = t < t_g
            if occ is not None:
                keep &= t < _ray_box(O, dirs, occ)
            ray_ids.append(ids[keep])
            ts_hit.append(t[keep])
            cone_ids.append(np.full(int(keep.sum()), i))

        npts = np.zeros(len(cx), dtype=int)
        nring = np.zeros(len(cx), dtype=int)
        if not ray_ids:
            return npts, nring
        ray_ids = np.concatenate(ray_ids)
        ts_hit = np.concatenate(ts_hit)
        cone_ids = np.concatenate(cone_ids)
        # A ray that meets several cones returns from the nearest (cones hide cones).
        order = np.lexsort((ts_hit, ray_ids))
        first = np.ones(len(order), dtype=bool)
        first[1:] = ray_ids[order][1:] != ray_ids[order][:-1]
        win_cone = cone_ids[order][first]
        win_chan = ray_ids[order][first] // n_h
        np.add.at(npts, win_cone, 1)
        pairs = np.unique(np.stack([win_cone, win_chan], axis=1), axis=0)
        np.add.at(nring, pairs[:, 0], 1)
        return npts, nring

    def _accumulate(self, npts, nring, view, rng):
        det = (npts >= self.min_points) & (nring >= self.min_rings)
        self.scans += 1
        if self.record:
            self._frames.append((npts.astype(np.int16), nring.astype(np.int16),
                                 np.where(view, rng, np.inf).astype(np.float32)))
        self.in_view += view
        self.hit += npts > 0
        self.detected += det
        self.max_points = np.maximum(self.max_points, npts)
        self.max_rings = np.maximum(self.max_rings, nring)
        with np.errstate(invalid='ignore'):
            self.first_detect_range = np.where(
                det, np.fmax(self.first_detect_range, rng), self.first_detect_range)
            self.closest_in_view = np.where(
                view, np.fmin(self.closest_in_view, rng), self.closest_in_view)

        b = np.digitize(rng, RANGE_BINS) - 1
        ok = view & (b >= 0) & (b < len(self.bin_in_view))
        np.add.at(self.bin_in_view, b[ok], 1)
        np.add.at(self.bin_detected, b[ok & det], 1)
        np.add.at(self.bin_points, b[ok], npts[ok])
        return int(det.sum()), int(view.sum())

    def frames(self):
        """Recorded scans: points, channels (int16) and range from the lidar (float32,
        inf when out of view), each (scans, cones)."""
        if not self._frames:
            n = len(self.cones)
            return (np.zeros((0, n), np.int16), np.zeros((0, n), np.int16),
                    np.zeros((0, n), np.float32))
        return tuple(np.stack(a) for a in zip(*self._frames))

    # ------------------------------------------------------------------ #
    def status(self):
        """Per cone: 'detected' | 'hit_only' | 'missed' | 'never_in_view'."""
        st = np.full(len(self.cones), 'never_in_view', dtype=object)
        st[self.in_view > 0] = 'missed'
        st[self.hit > 0] = 'hit_only'
        st[self.detected > 0] = 'detected'
        return st

    def summary(self):
        st = self.status()
        n = len(self.cones)
        per_color = {}
        for code, name in COLOR_NAMES.items():
            m = self.cones[:, 2] == code
            if m.any():
                per_color[name] = {'total': int(m.sum()),
                                   'detected': int((st[m] == 'detected').sum())}
        fdr = self.first_detect_range[~np.isnan(self.first_detect_range)]
        recall = []
        for i in range(len(self.bin_in_view)):
            v = int(self.bin_in_view[i])
            recall.append({
                'range_m': f'{RANGE_BINS[i]}-{RANGE_BINS[i + 1]}',
                'cone_scans_in_view': v,
                'detected_fraction': round(self.bin_detected[i] / v, 4) if v else None,
                'mean_points': round(self.bin_points[i] / v, 2) if v else None,
            })
        return {
            'scans': self.scans,
            'cones': n,
            'detected': int((st == 'detected').sum()),
            'hit_only': int((st == 'hit_only').sum()),
            'missed': int((st == 'missed').sum()),
            'never_in_view': int((st == 'never_in_view').sum()),
            'detected_fraction': round(float((st == 'detected').sum()) / n, 4) if n else None,
            'first_detect_range_m': {
                'median': round(float(np.median(fdr)), 2) if len(fdr) else None,
                'p10': round(float(np.percentile(fdr, 10)), 2) if len(fdr) else None,
                'max': round(float(fdr.max()), 2) if len(fdr) else None,
            },
            'cones_touched_by_car': int(self.touched.sum()) if self.body_box is not None else None,
            'per_color': per_color,
            'recall_by_range': recall,
            'criteria': {'min_points': self.min_points, 'min_rings': self.min_rings,
                         'catch_radius_m': self.r},
        }

    def cone_rows(self):
        st = self.status()
        rows = []
        for i, (x, y, c) in enumerate(self.cones):
            rows.append({
                'id': i, 'color': COLOR_NAMES.get(int(c), str(int(c))),
                'x': round(float(x), 3), 'y': round(float(y), 3), 'status': st[i],
                'scans_in_view': int(self.in_view[i]), 'scans_hit': int(self.hit[i]),
                'scans_detected': int(self.detected[i]),
                'max_points': int(self.max_points[i]), 'max_rings': int(self.max_rings[i]),
                'first_detect_range_m': None if np.isnan(self.first_detect_range[i])
                else round(float(self.first_detect_range[i]), 2),
                'closest_in_view_m': None if np.isnan(self.closest_in_view[i])
                else round(float(self.closest_in_view[i]), 2),
                'touched_by_car': bool(self.touched[i]),
            })
        return rows


def _ray_frustum(O, D, cx, cy, r0, r1, h):
    """Distance along each ray (origin O, unit directions D) to a vertical frustum
    standing on z = 0 at (cx, cy): radius r0 at the bottom, r1 at height h.
    Side surface only; inf where the ray misses."""
    k = (r0 - r1) / h
    ox, oy, oz = O[0] - cx, O[1] - cy, O[2]
    dx, dy, dz = D[:, 0], D[:, 1], D[:, 2]
    rz = r0 - k * oz                       # radius at the origin's height
    a = dx * dx + dy * dy - k * k * dz * dz
    b = 2 * (ox * dx + oy * dy) + 2 * k * dz * rz
    c = ox * ox + oy * oy - rz * rz
    disc = b * b - 4 * a * c
    t = np.full(len(D), np.inf)
    ok = (disc >= 0) & (np.abs(a) > 1e-12)
    sq = np.sqrt(np.where(ok, disc, 0.0))
    for sign in (-1.0, 1.0):
        with np.errstate(divide='ignore', invalid='ignore'):
            tr = (-b + sign * sq) / (2 * a)
        z = oz + tr * dz
        good = ok & (tr > 1e-6) & (z >= 0) & (z <= h) & (r0 - k * z >= 0)
        t = np.where(good & (tr < t), tr, t)
    return t


def _ray_box(O, D, box):
    """Entry distance of each ray into the box x0<x<x1, |y|<yh, 0<z<z1 (inf if missed)."""
    x0, x1, yh, z1 = box[:4]
    lo = np.array([x0, -yh, 0.0])
    hi = np.array([x1, yh, z1])
    with np.errstate(divide='ignore', invalid='ignore'):
        inv = 1.0 / D
        t1 = (lo - O) * inv
        t2 = (hi - O) * inv
    tmin = np.nanmax(np.minimum(t1, t2), axis=1)
    tmax = np.nanmin(np.maximum(t1, t2), axis=1)
    return np.where((tmax >= tmin) & (tmax > 0), np.maximum(tmin, 0.0), np.inf)
