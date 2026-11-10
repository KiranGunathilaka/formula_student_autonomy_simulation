"""
Cone EKF-SLAM: odometry increments + lidar cone detections -> car pose and cone map.

State [x, y, yaw, m1x, m1y, m2x, m2y, ...] in the map frame, one joint covariance.

  predict  per lidar scan, the odometry increment since the last scan (dx, dy, dyaw
           in the previous body frame), with noise that grows with distance and turn.
  update   cones detected in that scan (x, y in base_footprint, no colour):
           1. every mapped cone near the car is predicted into the car frame, with its
              innovation covariance S = H P H' + R;
           2. detections and mapped cones are paired by global nearest neighbour on
              the Mahalanobis distance (Hungarian assignment), pairs beyond the
              chi-square gate, or with the detection more than `max_innovation_m` from
              where the mapped cone should be, are dropped.  After a lap the car's
              uncertainty relative to the cones mapped at the start can reach metres,
              wide enough for the gate alone to pair a cone the map does not hold yet
              with its neighbour one cone spacing on: the car then jumps 3-5 m at the
              loop closure (seen with a detector that finds cones only within ~6 m);
           3. one EKF update with all pairs;
           4. a detection that pairs with nothing and is not within `min_separation`
              of a mapped cone becomes a candidate; a candidate joins the map, with
              its full cross-covariance, once it was seen in `candidate_min_obs`
              scans (each within `candidate_timeout_s` of the last), from car
              positions at least `candidate_baseline_m` apart, and its sightings
              stayed within `candidate_spread_m` (+ the tangential noise) of each
              other and within `candidate_spread_per_m` times the distance the car
              moved.  A static cone stays put while the car moves; returns that move
              with the car spread with it (Gazebo's gpu_ray floats ground returns near
              the horizon of long-range lidars, ~160 m for the ATX) and are never
              mapped.
Measurement noise is radial / tangential to the lidar: the detector sees only the near
face of a cone (radial error), and the beams spread with range (tangential error).
"""

import math

import numpy as np
from scipy.optimize import linear_sum_assignment

CHI2_2DOF_999 = 13.82


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class ConeSLAM:
    def __init__(self, pose0=(0.0, 0.0, 0.0), *, pose0_sigma=(0.01, 0.01, 0.002),
                 odom_sigma_abs=(0.005, 0.005, 0.0005),   # m, m, rad per step
                 odom_sigma_rel=(0.02, 0.01, 0.02),       # of |dx|, |dx|, |dyaw|
                 yaw_sigma_per_m=0.0005,                  # rad per m driven
                 range_sigma=0.10, tangential_sigma=0.05, tangential_sigma_per_m=0.002,
                 gate=CHI2_2DOF_999, max_innovation_m=1.0, min_separation=1.0, candidate_radius=0.6,
                 candidate_min_obs=3, candidate_timeout_s=1.0, candidate_baseline_m=1.0,
                 candidate_spread_m=0.25, candidate_spread_per_m=0.15, sensor_xy=(0.0, 0.0)):
        self.oa = np.asarray(odom_sigma_abs, float)
        self.orel = np.asarray(odom_sigma_rel, float)
        self.yaw_per_m = float(yaw_sigma_per_m)
        self.sr, self.st, self.st_m = float(range_sigma), float(tangential_sigma), \
            float(tangential_sigma_per_m)
        self.gate = float(gate)
        self.max_innov = float(max_innovation_m)
        self.min_sep = float(min_separation)
        self.cand_r = float(candidate_radius)
        self.cand_n = int(candidate_min_obs)
        self.cand_dt = float(candidate_timeout_s)
        self.cand_base = float(candidate_baseline_m)
        self.cand_spread = float(candidate_spread_m)
        self.cand_spread_per_m = float(candidate_spread_per_m)
        self.sensor = np.asarray(sensor_xy, float)
        cap = 3 + 2 * 256
        self.mu = np.zeros(cap)
        self.P = np.zeros((cap, cap))
        self.n = 3
        self.mu[:3] = pose0
        self.P[:3, :3] = np.diag(np.square(pose0_sigma))
        self.lm_obs = []          # times each landmark was matched (incl. its first)
        self.cands = []           # dicts: xy, count, last_t, z, seen
        self.scans = 0

    # ------------------------------------------------------------------ #
    @property
    def pose(self):
        return self.mu[0], self.mu[1], self.mu[2]

    @property
    def pose_cov(self):
        return self.P[:3, :3].copy()

    @property
    def landmarks(self):
        return self.mu[3:self.n].reshape(-1, 2).copy()

    def landmark_cov(self, i):
        c = 3 + 2 * i
        return self.P[c:c + 2, c:c + 2].copy()

    # ------------------------------------------------------------------ #
    def predict(self, dx, dy, dyaw):
        n = self.n
        x, y, th = self.mu[:3]
        c, s = math.cos(th), math.sin(th)
        self.mu[0] = x + c * dx - s * dy
        self.mu[1] = y + s * dx + c * dy
        self.mu[2] = wrap(th + dyaw)
        F = np.array([[1.0, 0.0, -s * dx - c * dy],
                      [0.0, 1.0, c * dx - s * dy],
                      [0.0, 0.0, 1.0]])
        G = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        dist = math.hypot(dx, dy)
        sig = self.oa + self.orel * np.array([dist, dist, abs(dyaw)])
        sig[2] += self.yaw_per_m * dist
        P = self.P
        P[:3, :3] = F @ P[:3, :3] @ F.T + G @ np.diag(sig ** 2) @ G.T
        if n > 3:
            P[:3, 3:n] = F @ P[:3, 3:n]
            P[3:n, :3] = P[:3, 3:n].T

    # ------------------------------------------------------------------ #
    def _meas_cov(self, z):
        """(J, 2, 2) noise of detections z (J, 2) in the car frame."""
        rel = z - self.sensor
        rng = np.maximum(np.hypot(rel[:, 0], rel[:, 1]), 1e-3)
        u = rel / rng[:, None]
        w = np.column_stack([-u[:, 1], u[:, 0]])
        st = self.st + self.st_m * rng
        return (self.sr ** 2 * np.einsum('ji,jk->jik', u, u)
                + (st ** 2)[:, None, None] * np.einsum('ji,jk->jik', w, w))

    def update(self, t, z):
        """Detections z (J, 2) in the car frame at time t. Returns per-scan stats."""
        self.scans += 1
        z = np.asarray(z, float).reshape(-1, 2)
        J = len(z)
        stats = {'detections': J, 'matched': 0, 'new': 0, 'nis': 0.0}
        if J == 0:
            self._expire(t)
            stats.update(landmarks=(self.n - 3) // 2, candidates=len(self.cands))
            return stats
        Rz = self._meas_cov(z)
        matched_j = np.zeros(J, bool)
        nl = (self.n - 3) // 2
        if nl:
            pairs, Hp, nis = self._associate(z, Rz)
            if pairs:
                self._batch_update(z, Rz, pairs, Hp)
                for j, k in pairs:
                    matched_j[j] = True
                    self.lm_obs[k] += 1
                stats['matched'] = len(pairs)
                stats['nis'] = float(np.mean(nis))
        stats['new'] = self._candidates(t, z[~matched_j], Rz[~matched_j])
        stats.update(landmarks=(self.n - 3) // 2, candidates=len(self.cands))
        return stats

    def _associate(self, z, Rz):
        n = self.n
        x, y, th = self.mu[:3]
        M = self.mu[3:n].reshape(-1, 2)
        d = M - np.array([x, y])
        reach = float(np.max(np.hypot(z[:, 0], z[:, 1]))) + 5.0
        idx = np.nonzero(np.hypot(d[:, 0], d[:, 1]) < reach)[0]
        if len(idx) == 0:
            return [], None, None
        c, s = math.cos(th), math.sin(th)
        RT = np.array([[c, s], [-s, c]])
        dRT = np.array([[-s, c], [-c, -s]])
        dk = d[idx]
        h = dk @ RT.T                                    # (K, 2)
        K = len(idx)
        Hp = np.zeros((K, 2, 3))
        Hp[:, :, :2] = -RT
        Hp[:, :, 2] = dk @ dRT.T
        cols = 3 + 2 * idx
        cc = cols[:, None] + np.arange(2)                # (K, 2)
        P = self.P
        Ppp = P[:3, :3]
        Ppk = np.transpose(P[:3][:, cc], (1, 0, 2))      # (K, 3, 2)
        Pkk = P[cc[:, :, None], cc[:, None, :]]          # (K, 2, 2)
        HpPpk = Hp @ Ppk                                 # (K, 2, 2)
        cross = HpPpk @ RT.T
        S0 = Hp @ Ppp @ np.transpose(Hp, (0, 2, 1)) + cross + np.transpose(cross, (0, 2, 1)) \
            + RT @ Pkk @ RT.T
        S = S0[None] + Rz[:, None]                       # (J, K, 2, 2)
        nu = z[:, None, :] - h[None]                     # (J, K, 2)
        a, b, cq, dq = S[..., 0, 0], S[..., 0, 1], S[..., 1, 0], S[..., 1, 1]
        det = a * dq - b * cq
        n0, n1 = nu[..., 0], nu[..., 1]
        d2 = (dq * n0 * n0 - (b + cq) * n0 * n1 + a * n1 * n1) / det
        d2 = np.where(np.hypot(n0, n1) > self.max_innov, np.inf, d2)
        cost = np.where(d2 < self.gate, d2, 1e6)
        rows, colk = linear_sum_assignment(cost)
        keep = cost[rows, colk] < self.gate
        pairs = [(int(j), int(idx[k])) for j, k in zip(rows[keep], colk[keep])]
        local = {int(idx[k]): int(k) for k in colk[keep]}
        return pairs, {lm: Hp[local[lm]] for _, lm in pairs}, d2[rows[keep], colk[keep]]

    def _batch_update(self, z, Rz, pairs, Hp):
        """One EKF update with every (detection j, landmark k) pair; Hp[k] is the
        pose part of that landmark's Jacobian.  H is sparse (the pose block and the
        landmark's own 2x2 block), so P H' and S are built from those blocks; only the
        covariance update itself touches all of P."""
        n = self.n
        x, y, th = self.mu[:3]
        c, s = math.cos(th), math.sin(th)
        RT = np.array([[c, s], [-s, c]])
        js = np.array([j for j, _ in pairs])
        ks = np.array([k for _, k in pairs])
        m = len(pairs)
        cols = 3 + 2 * ks
        ci = (cols[:, None] + np.arange(2)).ravel()                # (2m,)
        Hps = np.concatenate([Hp[k] for k in ks])                  # (2m, 3)
        lm = self.mu[ci].reshape(m, 2) - (x, y)
        nu = (z[js] - lm @ RT.T).ravel()
        Rb = np.zeros((2 * m, 2 * m))
        for i, j in enumerate(js):
            Rb[2 * i:2 * i + 2, 2 * i:2 * i + 2] = Rz[j]
        P = self.P[:n, :n]
        PHt = P[:, :3] @ Hps.T + (P[:, ci].reshape(n, m, 2) @ RT.T).reshape(n, 2 * m)
        S = Hps @ PHt[:3] + (RT @ PHt[ci].reshape(m, 2, 2 * m)).reshape(2 * m, 2 * m) + Rb
        Kg = np.linalg.solve(S, PHt.T).T
        self.mu[:n] += Kg @ nu
        self.mu[2] = wrap(self.mu[2])
        P -= Kg @ PHt.T
        self.P[:n, :n] = 0.5 * (P + P.T)

    # ------------------------------------------------------------------ #
    def _expire(self, t):
        self.cands = [c for c in self.cands if t - c['last_t'] <= self.cand_dt]

    def _candidates(self, t, z, Rz):
        self._expire(t)
        if len(z) == 0:
            return 0
        x, y, th = self.mu[:3]
        c, s = math.cos(th), math.sin(th)
        Rw = np.array([[c, -s], [s, c]])
        world = z @ Rw.T + (x, y)
        lms = self.mu[3:self.n].reshape(-1, 2)
        for cand in self.cands:
            cand['seen'] = False
        added = 0
        for j, w in enumerate(world):
            if len(lms) and np.min(np.hypot(*(lms - w).T)) < self.min_sep:
                continue                                  # a mapped cone, just not paired
            best, best_d = None, self.cand_r
            for cand in self.cands:
                dd = math.hypot(*(cand['xy'] - w))
                if not cand['seen'] and dd < best_d:
                    best, best_d = cand, dd
            if best is None:
                self.cands.append({'xy': w.copy(), 'count': 1, 'last_t': t, 'z': z[j],
                                   'R': Rz[j], 'seen': True, 'pts': [w.copy()],
                                   'car0': np.array([x, y])})
                continue
            best['count'] += 1
            best['xy'] += (w - best['xy']) / best['count']
            best['pts'].append(w.copy())
            best['last_t'], best['z'], best['R'], best['seen'] = t, z[j], Rz[j], True

        def ready_(cand):
            if cand['count'] < self.cand_n or not cand['seen']:
                return False
            base = math.hypot(x - cand['car0'][0], y - cand['car0'][1])
            if base < self.cand_base:
                return False
            pts = np.array(cand['pts'])
            spread = float(np.sqrt(np.sum(pts.var(axis=0))))
            rng = math.hypot(*cand['z'])
            return spread <= min(self.cand_spread + self.st_m * rng,
                                 self.cand_spread_per_m * base)
        ready = [cand for cand in self.cands if ready_(cand)]
        for cand in ready:
            lms = self.mu[3:self.n].reshape(-1, 2)
            w = Rw @ cand['z'] + (x, y)
            if len(lms) and np.min(np.hypot(*(lms - w).T)) < self.min_sep:
                continue
            self._add_landmark(cand['z'], cand['R'])
            added += 1
        done = {id(cand) for cand in ready}
        self.cands = [cand for cand in self.cands if id(cand) not in done]
        return added

    def _add_landmark(self, z, Rz):
        n = self.n
        if n + 2 > len(self.mu):
            cap = 2 * len(self.mu)
            mu, P = np.zeros(cap), np.zeros((cap, cap))
            mu[:n], P[:n, :n] = self.mu[:n], self.P[:n, :n]
            self.mu, self.P = mu, P
        x, y, th = self.mu[:3]
        c, s = math.cos(th), math.sin(th)
        Rw = np.array([[c, -s], [s, c]])
        Gp = np.array([[1.0, 0.0, -s * z[0] - c * z[1]],
                       [0.0, 1.0, c * z[0] - s * z[1]]])
        P = self.P
        self.mu[n:n + 2] = Rw @ z + (x, y)
        Pmx = Gp @ P[:3, :n]
        P[n:n + 2, :n] = Pmx
        P[:n, n:n + 2] = Pmx.T
        P[n:n + 2, n:n + 2] = Gp @ P[:3, :3] @ Gp.T + Rw @ Rz @ Rw.T
        self.n = n + 2
        self.lm_obs.append(1)
