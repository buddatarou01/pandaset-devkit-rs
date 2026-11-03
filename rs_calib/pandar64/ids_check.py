"""Beam IDs of every point with the joint solution (joint_pose_elev.py): mount, 64 calibrated elevations, pose corrections. Reports how
cleanly each point sits on one channel, the firing-pair check (B.4), and what the unassigned points are. <scene> <f0> <f1>"""
import os
HERE = os.path.dirname(os.path.abspath(__file__)); SP = os.environ.get('P64OUT', HERE)   # tables beside this file; results to P64OUT (default: here)
import sys, json, numpy as np, pandas as pd
from pandaset.geometry import _heading_position_to_mat
from scipy.spatial.transform import Rotation as R, Slerp
s, f0, f1 = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
Z = np.load(f'{SP}/pose_elev_{s}_{f0}_{f1}.npz'); x = Z['x']; CF = list(Z['CF']); NC = len(CF); mnt, E, cc = x[:5], x[5:69], x[69:].reshape(NC, 3)
T = np.loadtxt(f'{HERE}/p64_firing.txt'); PAIR = {}
for dt in np.unique(T[:, 2]):
    a_, b_ = T[np.isclose(T[:, 2], dt), 1].astype(int); PAIR[a_] = b_; PAIR[b_] = a_
D = f"{os.environ.get('PANDASET_ROOT', 'pandaset')}/{s}/lidar"; Pj = json.load(open(f'{D}/poses.json')); TS = np.array(json.load(open(f'{D}/timestamps.json')))
Ls = np.stack([np.asarray(_heading_position_to_mat(p['heading'], p['position'])) for p in Pj])
Rc = np.stack([Ls[CF[i], :3, :3] @ R.from_euler('xy', cc[i, :2], degrees=True).as_matrix() for i in range(NC)]); Pc = Ls[CF, :3, 3] + np.c_[np.zeros((NC, 2)), cc[:, 2]]
tot = dict(n=0, ok=0, pair=0, pairok=0); margins = []; offs = []; rr_bad = []
for f in range(f0, f1 + 1):
    d = pd.read_pickle(f'{D}/{f:02d}.pkl'); d = d[d.d == 0]; X = d[['x', 'y', 'z']].to_numpy(); tsec = d.t.to_numpy(); tw = np.clip(tsec, TS[CF[0]], TS[CF[-1]])
    j1 = np.clip(np.searchsorted(TS[CF], tw), 1, NC - 1); j0 = j1 - 1; a = (tw - TS[CF][j0]) / (TS[CF][j1] - TS[CF][j0]); q = np.empty_like(X)
    for k in range(NC - 1):
        m = j0 == k
        if m.any():
            Rm = Slerp([0, 1], R.from_matrix(Rc[k:k + 2]))(a[m]).as_matrix(); p = Pc[k] + a[m, None] * (Pc[k + 1] - Pc[k]); q[m] = np.einsum('nji,nj->ni', Rm, X[m] - p)
    v = (q - mnt[:3]) @ R.from_euler('xy', mnt[3:5], degrees=True).as_matrix(); r = np.linalg.norm(v, axis=1); el = np.degrees(np.arcsin(v[:, 2] / r))
    dd = np.abs(el[:, None] - E[None]); o = np.argsort(dd, 1); ch = o[:, 0] + 1; d1 = dd[np.arange(len(el)), o[:, 0]]; d2 = dd[np.arange(len(el)), o[:, 1]]
    ok = d1 < 0.06; tot['n'] += len(el); tot['ok'] += ok.sum(); margins.append(d2 - d1); offs.append(d1); rr_bad.append(r[~ok])
    u, inv, c = np.unique(tsec, return_inverse=True, return_counts=True); oo = np.argsort(inv, kind='stable'); st = np.r_[0, np.cumsum(c)[:-1]]
    g = np.nonzero(c == 2)[0]; i, j = oo[st[g]], oo[st[g] + 1]; b = ok[i] & ok[j]
    tot['pair'] += b.sum(); tot['pairok'] += sum(PAIR.get(int(ch[p]), -1) == int(ch[q_]) for p, q_ in zip(i[b], j[b]))
mg, of, rb = np.concatenate(margins), np.concatenate(offs), np.concatenate(rr_bad)
print(f'{s} frames {f0}-{f1}: {tot["n"]} points | within 0.06 deg of a channel: {100 * tot["ok"] / tot["n"]:.2f}% | distance to nearest channel: median {np.median(of):.4f}, p99 {np.percentile(of, 99):.4f} deg')
print(f'   margin (2nd nearest - nearest): p1 {np.percentile(mg, 1):.3f}, median {np.median(mg):.3f} deg (channels 0.165-6 deg apart)')
print(f'   same-time pairs, both assigned: {tot["pair"]} | = a B.4 firing pair: {100 * tot["pairok"] / tot["pair"]:.3f}%')
print(f'   unassigned points: {len(rb)} | their range: < 1.5 m {np.mean(rb < 1.5) * 100:.1f}%, 1.5-3 m {np.mean((rb >= 1.5) & (rb < 3)) * 100:.1f}%, > 3 m {np.mean(rb >= 3) * 100:.1f}%')
