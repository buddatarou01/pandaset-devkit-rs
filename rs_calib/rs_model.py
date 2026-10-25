"""Rolling-shutter-aware LiDAR -> camera projection for PandaSet.

Model (one set of timing per light group, the rest per camera; values in rs_params.json):
  - timing: row v of an image is exposed at  t_image + delta - readout * (1079 - v) / 1079  (ms; the BOTTOM row is at the image
    timestamp + delta, the top row `readout` earlier). readout / delta are shared by all cameras of a light group
    (day / dusk / night, from the local time of the scene's first front-camera timestamp).
  - ego at that time: the front-camera pose (poses.json) at the image timestamp, moved with a constant velocity (central
    difference of the front poses around it).
  - extrinsic: the shipped extrinsic (camera -> front camera, mean over frames of the poses.json camera poses) with a small
    correction in the camera frame: position shift (tx, ty, tz cm) and yaw / pitch / roll (deg, yaw about camera y, pitch about x,
    roll about z).
  - intrinsics: shipped fx, fy, cx, cy with a principal-point shift (dcx, dcy px); lens pinhole or Kannala-Brandt 4 terms
    (theta_d = theta (1 + k1 th^2 + k2 th^4 + k3 th^6 + k4 th^8), radius = f theta_d).
Each LiDAR point is projected with the camera pose at the time of the row it lands on (3 fixed-point passes).
"""
from __future__ import annotations

import datetime
import json
import zoneinfo

import numpy as np
from scipy.spatial.transform import Rotation as R

from pandaset_io import Scene, distributed_extrinsics, ego_at, inv

W, H = 1920, 1080
KB_TAN = (1 / 3, 2 / 15, 17 / 315, 62 / 2835)          # tan(theta) series: KB4 with these k equals the pinhole to ~1 px at 46 deg


def light_group(sc: Scene) -> str:
    """day / dusk / night from the first front-camera timestamp in US Pacific time (PandaSet: 2019-05-10 18:58-19:47 dusk,
    21:50-22:41 night, 2019-05-11 09:35-10:50 day)."""
    t = datetime.datetime.fromtimestamp(sc.stamps('front_camera')[0], zoneinfo.ZoneInfo('America/Los_Angeles'))
    return 'day' if t.hour < 12 else ('night' if t.hour >= 21 else 'dusk')


def load_params(path: str, sc: Scene, cam: str) -> dict:
    """Flat parameter dict for one scene / camera from rs_params.json (missing entries = no correction, pinhole)."""
    P = json.load(open(path)); c = P.get('cameras', {}).get(cam, {}); T = P.get('timing', {})
    tm = T if 'readout' in T else T.get(light_group(sc), {})
    return dict(readout=float(tm.get('readout', 0.0)), delta=float(tm.get('delta', 0.0)),
                yaw=c.get('yaw', 0.0), pitch=c.get('pitch', 0.0), roll=c.get('roll', 0.0),
                tx=c.get('tx_cm', 0.0), ty=c.get('ty_cm', 0.0), tz=c.get('tz_cm', 0.0), dcx=c.get('dcx_px', 0.0), dcy=c.get('dcy_px', 0.0),
                lens=c.get('lens', 'pinhole'), **{f'k{i + 1}': c.get(f'k{i + 1}', KB_TAN[i]) for i in range(4)})


def to_pixel(q: np.ndarray, K: np.ndarray, lens: dict | None = None) -> np.ndarray:
    """Camera-frame points (N, 3) -> pixels (N, 2) with the pinhole or the KB4 lens."""
    if not lens or lens.get('model', 'pinhole') != 'kb4':
        return (q @ K.T)[:, :2] / np.maximum(q[:, 2:3], 1e-9)
    r = np.linalg.norm(q[:, :2], axis=1); th = np.arctan2(r, q[:, 2]); t2 = th * th
    td = th * (1 + t2 * (lens['k1'] + t2 * (lens['k2'] + t2 * (lens['k3'] + t2 * lens['k4']))))
    s = np.where(r > 1e-12, td / np.maximum(r, 1e-12), 1.0 / np.maximum(q[:, 2], 1e-9))
    return np.c_[K[0, 0] * q[:, 0] * s + K[0, 2], K[1, 1] * q[:, 1] * s + K[1, 2]]


def lens_valid(q: np.ndarray, K: np.ndarray, lens: dict | None = None, margin_deg: float = 3.0) -> np.ndarray:
    """Points the lens may draw: in front (z > 0.3 m); for KB4 also inside the range where theta_d still increases and not more than
    margin_deg past the farthest image corner (beyond that the polynomial folds back and sideways points land in the image)."""
    ok = q[:, 2] > 0.3
    if not lens or lens.get('model', 'pinhole') != 'kb4':
        return ok
    k = [lens['k1'], lens['k2'], lens['k3'], lens['k4']]
    tg = np.linspace(0, np.pi / 2, 4001); t2 = tg * tg
    tdg = tg * (1 + t2 * (k[0] + t2 * (k[1] + t2 * (k[2] + t2 * k[3]))))
    dec = np.nonzero(np.diff(tdg) <= 0)[0]; th_mono = tg[dec[0]] if len(dec) else np.pi / 2
    r_corner = max(np.hypot(u - K[0, 2], v - K[1, 2]) for u in (0, W - 1) for v in (0, H - 1)) / (0.5 * (K[0, 0] + K[1, 1]))
    inside = np.nonzero((tdg >= r_corner) & (tg <= th_mono))[0]; th_corner = tg[inside[0]] if len(inside) else th_mono
    th = np.arctan2(np.linalg.norm(q[:, :2], axis=1), q[:, 2])
    return ok & (th < th_mono) & (th < th_corner + np.radians(margin_deg))


class Projector:
    """Projector(scene, cam, params).project(points_world, t) -> pixels (M, 2), depth (M,), index (M,) of the points drawn in frame t."""

    def __init__(self, sc: Scene, cam: str, p: dict):
        self.sc, self.cam, self.p = sc, cam, p
        X = distributed_extrinsics(sc)[cam].copy()
        dR = R.from_euler('yxz', [p['yaw'], p['pitch'], p['roll']], degrees=True).as_matrix()
        X[:3, 3] = X[:3, 3] + X[:3, :3] @ (np.array([p['tx'], p['ty'], p['tz']]) / 100); X[:3, :3] = X[:3, :3] @ dR
        self.X = X; self.K = sc.K(cam).copy(); self.K[0, 2] += p['dcx']; self.K[1, 2] += p['dcy']
        self.lens = dict(model=p['lens'], **{f'k{i + 1}': p[f'k{i + 1}'] for i in range(4)})
        self.T, self.ts = sc.T_wc_all('front_camera'), sc.stamps('front_camera')

    def _ego_motion(self, t: int):
        tq = self.sc.stamps(self.cam)[t]; T, ts = self.T, self.ts
        i = int(np.clip(np.searchsorted(ts, tq), 1, len(ts) - 1)); j0, j1 = max(i - 1, 0), min(i + 1, len(ts) - 1)
        if j1 - j0 < 2: j0, j1 = max(0, j1 - 2), min(len(ts) - 1, j0 + 2)
        v = (T[j1][:3, 3] - T[j0][:3, 3]) / (ts[j1] - ts[j0])                                             # world, m/s
        w = R.from_matrix(T[j0][:3, :3].T @ T[j1][:3, :3]).as_rotvec() / (ts[j1] - ts[j0])              # front frame, rad/s
        return ego_at(self.sc, self.cam, t), v, w

    def camera_points(self, P: np.ndarray, t: int, rolling_shutter: bool = True) -> np.ndarray:
        """World points (N, 3) -> camera-frame points (N, 3), each in the camera frame of the row it lands on."""
        E0, v, w = self._ego_motion(t); Xi = inv(self.X); p = self.p
        ro, de = (p['readout'], p['delta']) if rolling_shutter else (0.0, 0.0)
        dt = np.full(len(P), de * 1e-3)
        for _ in range(3):
            Pe = (P - E0[:3, 3] - v[None] * dt[:, None]) @ E0[:3, :3]
            Pe = Pe - np.cross(w[None] * dt[:, None], Pe)                                                  # small-angle inverse rotation
            q = Pe @ Xi[:3, :3].T + Xi[:3, 3]
            vrow = np.where(lens_valid(q, self.K, self.lens), to_pixel(q, self.K, self.lens)[:, 1], 1079.0)
            dt = de * 1e-3 - np.clip((1079 - vrow) / 1079, 0, 1) * ro * 1e-3
        return q

    def project(self, P: np.ndarray, t: int, rolling_shutter: bool = True):
        q = self.camera_points(P, t, rolling_shutter); ok = lens_valid(q, self.K, self.lens); uv = to_pixel(q, self.K, self.lens)
        ok &= (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H); idx = np.nonzero(ok)[0]
        return uv[idx], q[idx, 2], idx
