"""All 64 Pandar64 beam IDs from the mount solved on the 4 isolated beams, checked against the firing table, then the mount re-solved
with every ID fixed. Points back to the poses.json frame with the pose at their own time. <scene> <frame> <x,y,z,rx,ry>"""
import os
HERE = os.path.dirname(os.path.abspath(__file__)); SP = os.environ.get('P64OUT', HERE)   # tables beside this file; results to P64OUT (default: here)
import sys, json, numpy as np, pandas as pd
from pandaset.geometry import _heading_position_to_mat
from scipy.spatial.transform import Rotation as R, Slerp
from scipy.optimize import least_squares
s, f = sys.argv[1], int(sys.argv[2]); M0 = np.array([float(x) for x in sys.argv[3].split(',')]); WIN = 0.06

A = np.loadtxt(f'{HERE}/p64_angles.txt'); ELV = A[:, 2]; T = np.loadtxt(f'{HERE}/p64_firing.txt')
PAIR = {}
for dt in np.unique(T[:, 2]):
    a, b = T[np.isclose(T[:, 2], dt), 1].astype(int); PAIR[a] = b; PAIR[b] = a
D = f"{os.environ.get('PANDASET_ROOT', 'pandaset')}/{s}/lidar"; Pj = json.load(open(f'{D}/poses.json')); TS = np.array(json.load(open(f'{D}/timestamps.json')))
Ls = np.stack([np.asarray(_heading_position_to_mat(p['heading'], p['position'])) for p in Pj])
d = pd.read_pickle(f'{D}/{f:02d}.pkl'); d = d[d.d == 0]; X = d[['x', 'y', 'z']].to_numpy(); tsec = d.t.to_numpy()
ii = [max(f - 1, 0), f, min(f + 1, len(Ls) - 1)]; tt = np.clip(tsec, TS[ii[0]], TS[ii[2]])
Rm = Slerp(TS[ii], R.from_matrix(Ls[ii, :3, :3]))(tt).as_matrix(); pos = np.stack([np.interp(tt, TS[ii], Ls[ii, k, 3]) for k in range(3)], 1)
q = np.einsum('nji,nj->ni', Rm, X - pos)
def elev(p, qq=q):
    v = (qq - p[:3]) @ R.from_euler('xy', p[3:5], degrees=True).as_matrix(); return np.degrees(np.arcsin(v[:, 2] / np.linalg.norm(v, axis=1))), np.linalg.norm(v, axis=1)
el, r = elev(M0); ch = np.argmin(np.abs(el[:, None] - ELV[None]), 1) + 1; dev = el - ELV[ch - 1]; ok = (np.abs(dev) < WIN) & (r > 1.5)
print(f'{s} t={f}: assigned within +-{WIN} deg: {ok.sum()} of {len(q)} points ({100 * ok.mean():.1f}%), channels with points: {len(np.unique(ch[ok]))} of 64')
u, inv, c = np.unique(tsec, return_inverse=True, return_counts=True); o = np.argsort(inv, kind='stable'); st = np.r_[0, np.cumsum(c)[:-1]]
g = np.nonzero(c == 2)[0]; a, b = o[st[g]], o[st[g] + 1]; both = ok[a] & ok[b]
match = np.array([PAIR.get(int(ch[i]), -1) == int(ch[j]) for i, j in zip(a[both], b[both])])
print(f'   same-time pairs with both points assigned: {both.sum()} | pair = a firing-table pair (B.4): {100 * match.mean():.1f}%')
sel = np.nonzero(ok)[0]; tgt = ELV[ch[sel] - 1]
res = lambda p: elev(p, q[sel])[0] - tgt
p = least_squares(res, M0, loss='soft_l1', f_scale=0.02).x; rr = res(p)
print(f'   re-solved with all IDs: x {p[0]:+.4f} y {p[1]:+.4f} z {p[2]:+.4f} m, rx {p[3]:+.4f} ry {p[4]:+.4f} deg | |residual| median {np.median(np.abs(rr)):.4f} deg, p90 {np.percentile(np.abs(rr), 90):.4f}')
el2, _ = elev(p); dev2 = el2 - ELV[ch - 1]
mad = [np.median(np.abs(dev2[ok & (ch == k)])) for k in range(1, 65) if np.sum(ok & (ch == k)) > 20]
print(f'   per-channel |elevation - design| median over channels: {np.median(mad):.4f} deg, worst channel {np.max(mad):.4f}')
np.save(f'{SP}/mount_all_{s}_{f}.npy', p)
