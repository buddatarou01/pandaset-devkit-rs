"""Minimal PandaSet access for the rolling-shutter tools: images, intrinsics, camera poses, timestamps, LiDAR.

Reads the files of one sequence directly (no full-sequence image decode). Pose convention is the devkit's: pose = camera ->
world (T_wc) from pandaset.geometry._heading_position_to_mat. LiDAR points are in world coordinates (columns x, y, z, i, t, d;
d = 0 mechanical 360-degree LiDAR, d = 1 forward-facing LiDAR). Intrinsics are fx, fy, cx, cy only.

The dataset location comes from the environment variable PANDASET_ROOT (a folder with one sub-folder per sequence, LiDAR and
annotation .pkl files uncompressed or .pkl.gz).
"""
from __future__ import annotations

import gzip
import json
import os
import pickle
from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R, Slerp

from pandaset.geometry import _heading_position_to_mat

ROOT = os.environ.get('PANDASET_ROOT', 'pandaset')
CAMS = ['front_camera', 'front_left_camera', 'front_right_camera', 'left_camera', 'right_camera', 'back_camera']


@lru_cache(maxsize=None)
def _json(path: str):
    return json.load(open(path))


def _pkl(stem: Path):
    """stem without suffix: reads stem.pkl, or stem.pkl.gz when only the compressed file exists."""
    p = stem.with_suffix('.pkl')
    if p.exists():
        return pd.read_pickle(p)
    with gzip.open(str(p) + '.gz', 'rb') as f:
        return pickle.load(f)


class Scene:
    def __init__(self, seq: str, root: str = ROOT):
        self.seq = seq
        self._dir = Path(root) / seq
        if not (self._dir / 'camera' / 'front_camera' / 'poses.json').exists():
            raise FileNotFoundError(self._dir)

    def _cam(self, cam: str, name: str):
        return _json(str(self._dir / 'camera' / cam / name))

    def n_frames(self, cam: str = 'front_camera') -> int:
        return len(self._cam(cam, 'timestamps.json'))

    def K(self, cam: str) -> np.ndarray:
        c = self._cam(cam, 'intrinsics.json')
        return np.array([[c['fx'], 0, c['cx']], [0, c['fy'], c['cy']], [0, 0, 1]], np.float64)

    def T_wc(self, cam: str, t: int) -> np.ndarray:
        p = self._cam(cam, 'poses.json')[t]
        return np.asarray(_heading_position_to_mat(p['heading'], p['position']), np.float64)

    def T_wc_all(self, cam: str) -> np.ndarray:
        return np.stack([self.T_wc(cam, t) for t in range(self.n_frames(cam))])

    def stamps(self, cam: str) -> np.ndarray:
        return np.asarray(self._cam(cam, 'timestamps.json'), np.float64)

    def image(self, cam: str, t: int) -> np.ndarray:
        """RGB uint8 (H, W, 3)."""
        p = self._dir / 'camera' / cam / f'{t:02d}.jpg'
        bgr = cv2.imread(str(p))
        if bgr is None:
            raise FileNotFoundError(p)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def lidar(self, t: int, sensor: int | None = 0) -> np.ndarray:
        """(N, 6) world-frame x, y, z, i, t, d for frame t; sensor 0 / 1 / None (both)."""
        df = _pkl(self._dir / 'lidar' / f'{t:02d}')
        if sensor is not None:
            df = df.loc[df['d'] == sensor]
        return df[['x', 'y', 'z', 'i', 't', 'd']].to_numpy(np.float64)


def inv(T: np.ndarray) -> np.ndarray:
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def interp_pose(Ts: np.ndarray, stamps: np.ndarray, tq: np.ndarray) -> np.ndarray:
    """Poses at query times: slerp on rotation, linear on translation (clamped at the ends). Ts (F, 4, 4) -> (Q, 4, 4)."""
    tq = np.clip(np.asarray(tq, np.float64), stamps[0], stamps[-1])
    out = np.tile(np.eye(4), (len(tq), 1, 1))
    out[:, :3, :3] = Slerp(stamps, R.from_matrix(Ts[:, :3, :3]))(tq).as_matrix()
    for a in range(3):
        out[:, a, 3] = np.interp(tq, stamps, Ts[:, a, 3])
    return out


def ego_at(sc: Scene, cam: str, t: int) -> np.ndarray:
    """Front-camera pose ("ego") at camera cam's timestamp of frame t: slerp / linear between the two front poses around it
    (extrapolated beyond the ends)."""
    ts = sc.stamps('front_camera'); tb = sc.stamps(cam)[t]
    f0 = int(np.clip(np.searchsorted(ts, tb) - 1, 0, len(ts) - 2))
    T0, T1 = sc.T_wc('front_camera', f0), sc.T_wc('front_camera', f0 + 1); s = (tb - ts[f0]) / (ts[f0 + 1] - ts[f0])
    E = np.eye(4); E[:3, :3] = T0[:3, :3] @ R.from_rotvec(s * R.from_matrix(T0[:3, :3].T @ T1[:3, :3]).as_rotvec()).as_matrix()
    E[:3, 3] = T0[:3, 3] + s * (T1[:3, 3] - T0[:3, 3])
    return E


def distributed_extrinsics(sc: Scene) -> dict:
    """Camera k -> front-camera frame (4x4) as shipped: the mean over frames of inv(front pose at camera k's timestamp) . pose_k."""
    Tf, tf = sc.T_wc_all('front_camera'), sc.stamps('front_camera')
    X = {}
    for cam in CAMS:
        Tk, tk = sc.T_wc_all(cam), sc.stamps(cam)
        ok = np.nonzero((tk >= tf[0]) & (tk <= tf[-1]))[0]
        E = interp_pose(Tf, tf, tk[ok])
        Xs = np.stack([inv(E[i]) @ Tk[t] for i, t in enumerate(ok)])
        M = np.eye(4); M[:3, :3] = R.from_matrix(Xs[:, :3, :3]).mean().as_matrix(); M[:3, 3] = Xs[:, :3, 3].mean(0)
        X[cam] = M
    return X
