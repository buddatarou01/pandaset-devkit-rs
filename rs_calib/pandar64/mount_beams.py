"""LiDAR mount from beam elevations: points back to the vehicle frame with the lidar pose (poses.json) at their own time; unknown sensor
position (x, y, z) and tilt (roll, pitch) in that frame; the top 5 / bottom 10 Pandar64 beams (Appendix A); each point assigned to the
nearest design elevation (re-assigned every round, window shrinking), robust least squares on (elevation - design). <scene> <frame>"""
import os
HERE = os.path.dirname(os.path.abspath(__file__)); SP = os.environ.get('P64OUT', HERE)   # tables beside this file; results to P64OUT (default: here)
import sys, json, numpy as np, pandas as pd
from pandaset.geometry import _heading_position_to_mat
from scipy.spatial.transform import Rotation as R, Slerp
from scipy.optimize import least_squares
s, f = sys.argv[1], int(sys.argv[2])
import os
BEAMS = np.array([float(x) for x in os.environ.get('BEAMS', '14.882 11.032 8.059 5.057 3.040 -7.063 -8.059 -9.060 -9.885 -11.032 -12.006 -12.974 -13.930 -18.889 -24.897').split()])
FIXXY = int(os.environ.get('FIXXY', '0'))   # 1: x, y held at the paper's figure (0, 0.725)
D = f"{os.environ.get('PANDASET_ROOT', 'pandaset')}/{s}/lidar"; Pj = json.load(open(f'{D}/poses.json')); TS = np.array(json.load(open(f'{D}/timestamps.json')))
Ls = np.stack([np.asarray(_heading_position_to_mat(p['heading'], p['position'])) for p in Pj])
d = pd.read_pickle(f'{D}/{f:02d}.pkl'); d = d[d.d == 0]; X = d[['x', 'y', 'z']].to_numpy(); tsec = d.t.to_numpy()
ii = [max(f - 1, 0), f, min(f + 1, len(Ls) - 1)]; tt = np.clip(tsec, TS[ii[0]], TS[ii[2]])
Rm = Slerp(TS[ii], R.from_matrix(Ls[ii, :3, :3]))(tt).as_matrix(); pos = np.stack([np.interp(tt, TS[ii], Ls[ii, k, 3]) for k in range(3)], 1)
q = np.einsum('nji,nj->ni', Rm, X - pos); q = q[np.linalg.norm(q - [0, 0.725, 1.7], axis=1) > 2]
def elev(p):
    v = (q - p[:3]) @ R.from_euler('xy', p[3:5], degrees=True).as_matrix(); return np.degrees(np.arcsin(v[:, 2] / np.linalg.norm(v, axis=1)))
p = np.array([0, 0.725, 1.70, 0, 0])
for win in (1.5, 1.0, 0.6, 0.4, 0.3, 0.3):
    el = elev(p); j = np.argmin(np.abs(el[:, None] - BEAMS[None]), 1); m = np.abs(el - BEAMS[j]) < win
    # the two lowest beams are 6 deg apart from the rest: let them in with a wider window at first
    m |= (BEAMS[j] < -15) & np.any(BEAMS < -15) & (np.abs(el - BEAMS[j]) < max(win, 3.0) if win > 0.6 else False)
    sel, bj = np.nonzero(m)[0], BEAMS[j[m]]
    full = lambda x: np.r_[0, 0.725, x] if FIXXY else x
    res = lambda x: elev(full(x))[sel] - bj
    p = full(least_squares(res, p[2:] if FIXXY else p, loss='soft_l1', f_scale=0.1).x)
    r_ = res(p); print(f'   window {win}: {len(sel)} points, |residual| median {np.median(np.abs(r_)):.3f} deg | x {p[0]:+.3f} y {p[1]:+.3f} z {p[2]:+.3f} m roll {p[3]:+.3f} pitch {p[4]:+.3f} deg')
el = elev(p); j = np.argmin(np.abs(el[:, None] - BEAMS[None]), 1)
print(f'{s} t={f}: sensor at x {p[0]:+.3f} y {p[1]:+.3f} z {p[2]:+.3f} m above the poses.json origin, roll {p[3]:+.3f} pitch {p[4]:+.3f} deg')
for b in BEAMS:
    m = (BEAMS[j] == b) & (np.abs(el - b) < 0.4)
    print(f'   beam {b:+7.3f}: {m.sum():5d} pts, median {np.median(el[m]) - b if m.any() else np.nan:+.3f} deg off, MAD {np.median(np.abs(el[m] - np.median(el[m]))) if m.any() else np.nan:.3f}')
np.save(f'{SP}/mount_{s}_{f}.npy', p)
