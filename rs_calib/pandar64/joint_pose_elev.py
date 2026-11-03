"""Mount + 64 channel elevations + per-frame pose corrections (roll, pitch, height: what elevations can see) over consecutive sweeps.
Each point's pose = the corrected lidar poses (poses.json x small rotation about x, y and a z shift at every frame) interpolated at its own
time; every point of channel k must have elevation e_k from the mount. Mean pose correction held at 0 (it is the mount's tilt otherwise).
Beam IDs from the solved 4-beam mount (+-0.06 deg). <scene> <first> <last> <x,y,z,rx,ry>"""
import os
HERE = os.path.dirname(os.path.abspath(__file__)); SP = os.environ.get('P64OUT', HERE)   # tables beside this file; results to P64OUT (default: here)
import sys, json, numpy as np, pandas as pd
from pandaset.geometry import _heading_position_to_mat
from scipy.spatial.transform import Rotation as R, Slerp
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
s, f0, f1 = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]); M0 = np.array([float(x) for x in sys.argv[4].split(',')]); NPF = 4000

ELV = np.loadtxt(f'{HERE}/p64_angles.txt')[:, 2]
D = f"{os.environ.get('PANDASET_ROOT', 'pandaset')}/{s}/lidar"; Pj = json.load(open(f'{D}/poses.json')); TS = np.array(json.load(open(f'{D}/timestamps.json')))
Ls = np.stack([np.asarray(_heading_position_to_mat(p['heading'], p['position'])) for p in Pj])
CF = list(range(max(f0 - 1, 0), f1 + 1)); NC = len(CF)   # a sweep's points lie between the previous frame's stamp and its own: no control after f1; NC = len(CF)     # control frames (poses corrected)
rng = np.random.default_rng(0); Xw, tw = [], []
for f in range(f0, f1 + 1):
    d = pd.read_pickle(f'{D}/{f:02d}.pkl'); d = d[d.d == 0]; Xw.append(d[['x', 'y', 'z']].to_numpy()); tw.append(d.t.to_numpy())
Xw, tw = np.concatenate(Xw), np.concatenate(tw); tw = np.clip(tw, TS[CF[0]], TS[CF[-1]])
j1 = np.clip(np.searchsorted(TS[CF], tw), 1, NC - 1); j0 = j1 - 1; a = (tw - TS[CF][j0]) / (TS[CF][j1] - TS[CF][j0])
def poses(c):                                   # c: (NC, 3) = rx, ry (deg), dz (m) per control frame
    Rc = np.stack([Ls[CF[i], :3, :3] @ R.from_euler('xy', c[i, :2], degrees=True).as_matrix() for i in range(NC)]); Pc = Ls[CF, :3, 3] + np.c_[np.zeros((NC, 2)), c[:, 2]]
    return Rc, Pc
def to_vehicle(c, idx):
    Rc, Pc = poses(c); out = np.empty((len(idx), 3))
    for k in range(NC - 1):
        m = j0[idx] == k
        if not m.any(): continue
        sl = Slerp([0, 1], R.from_matrix(Rc[k:k + 2])); Rm = sl(a[idx][m]).as_matrix(); p = Pc[k] + a[idx][m, None] * (Pc[k + 1] - Pc[k])
        out[m] = np.einsum('nji,nj->ni', Rm, Xw[idx][m] - p)
    return out
def elev(mnt, qq):
    v = (qq - mnt[:3]) @ R.from_euler('xy', mnt[3:5], degrees=True).as_matrix(); return np.degrees(np.arcsin(v[:, 2] / np.linalg.norm(v, axis=1))), np.linalg.norm(v, axis=1)
q0 = to_vehicle(np.zeros((NC, 3)), np.arange(len(Xw))); el, r = elev(M0, q0); ch = np.argmin(np.abs(el[:, None] - ELV[None]), 1); ok = (np.abs(el - ELV[ch]) < 0.06) & (r > 1.5)
sel = rng.choice(np.nonzero(ok)[0], min(ok.sum(), NPF * (f1 - f0 + 1)), replace=False); c = ch[sel]
print(f'{s} frames {f0}-{f1}: {len(Xw)} points, {ok.sum()} with an ID ({100 * ok.mean():.1f}%), {len(sel)} used, {NC} control poses')
def unpack(x): return x[:5], x[5:69], x[69:].reshape(NC, 3)
def res(x):
    mnt, e, cc = unpack(x); e1 = elev(mnt, to_vehicle(cc, sel))[0] - e[c]
    return np.r_[e1, 100 * cc.mean(0), 0.02 * (cc / [0.5, 0.5, 0.2]).ravel()]   # gauge: mean correction 0; weak prior 0.5 deg / 0.2 m per pose
J = lil_matrix((len(sel) + 3 + 3 * NC, 69 + 3 * NC)); J[:len(sel), :5] = 1; J[np.arange(len(sel)), 5 + c] = 1
for i in range(len(sel)):
    k = j0[sel[i]]; J[i, 69 + 3 * k:69 + 3 * k + 6] = 1
for k in range(3): J[len(sel) + k, 69 + k::3] = 1
for k in range(3 * NC): J[len(sel) + 3 + k, 69 + k] = 1
x0 = np.r_[M0, ELV, np.zeros(3 * NC)]; r0 = res(x0)[:len(sel)]
x = least_squares(res, x0, loss='soft_l1', f_scale=0.02, jac_sparsity=J, x_scale='jac').x; rr = res(x)[:len(sel)]; mnt, e, cc = unpack(x)
print(f'   |elevation residual| median {np.median(np.abs(r0)):.4f} -> {np.median(np.abs(rr)):.4f} deg, p90 {np.percentile(np.abs(r0), 90):.4f} -> {np.percentile(np.abs(rr), 90):.4f}')
print(f'   mount x {mnt[0]:+.4f} y {mnt[1]:+.4f} z {mnt[2]:+.4f} m, rx {mnt[3]:+.4f} ry {mnt[4]:+.4f} deg | channel elevation - datasheet median {np.median(e - ELV):+.4f} deg, range {np.min(e - ELV):+.4f}..{np.max(e - ELV):+.4f}')
for i, fr in enumerate(CF): print(f'   frame {fr:2d}: pose correction rx {cc[i, 0]:+.4f} ry {cc[i, 1]:+.4f} deg, dz {cc[i, 2] * 100:+.2f} cm')
np.savez(f'{SP}/pose_elev_{s}_{f0}_{f1}.npz', x=x, CF=CF)
