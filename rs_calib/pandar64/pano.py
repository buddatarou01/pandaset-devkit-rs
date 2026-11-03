"""Virtual camera at the LiDAR: 360 x elevation panorama (black, 1 px dots) of one PandaSet sweep, each point back to the vehicle frame
with the lidar pose (poses.json) at its own time, seen from (a) the poses.json origin and (b) the LiDAR at (0, Y0, H). The top 5 and
bottom 10 Pandar64 beams (Appendix A) coloured by nearest design elevation within +-TOL, other points grey. <scene> <frame> <H> <out.png>"""
import os
HERE = os.path.dirname(os.path.abspath(__file__)); SP = os.environ.get('P64OUT', HERE)   # tables beside this file; results to P64OUT (default: here)
import sys, json, numpy as np, pandas as pd, cv2
from pandaset.geometry import _heading_position_to_mat
from scipy.spatial.transform import Rotation as R, Slerp
s, f, OUT = sys.argv[1], int(sys.argv[2]), sys.argv[3]; MNT = [float(x) for x in sys.argv[4].split(',')]; TOL = float(sys.argv[5]) if len(sys.argv) > 5 else 0.45   # mount x,y,z,rx,ry (vehicle frame, deg)
BEAMS = [14.882, 11.032, 8.059, 5.057, 3.040, -7.063, -8.059, -9.060, -9.885, -11.032, -12.006, -12.974, -13.930, -18.889, -24.897]
D = f"{os.environ.get('PANDASET_ROOT', 'pandaset')}/{s}/lidar"; Pj = json.load(open(f'{D}/poses.json')); TS = np.array(json.load(open(f'{D}/timestamps.json')))
Ls = np.stack([np.asarray(_heading_position_to_mat(p['heading'], p['position'])) for p in Pj])
d = pd.read_pickle(f'{D}/{f:02d}.pkl'); d = d[d.d == 0]; X = d[['x', 'y', 'z']].to_numpy(); tsec = d.t.to_numpy()
ii = [max(f - 1, 0), f, min(f + 1, len(Ls) - 1)]; tt = np.clip(tsec, TS[ii[0]], TS[ii[2]])
Rm = Slerp(TS[ii], R.from_matrix(Ls[ii, :3, :3]))(tt).as_matrix(); pos = np.stack([np.interp(tt, TS[ii], Ls[ii, k, 3]) for k in range(3)], 1)
q = np.einsum('nji,nj->ni', Rm, X - pos)
SX, SY, E0, E1 = 6, 20, 17, -27; W, Hh = 360 * SX, (E0 - E1) * SY
COL = [tuple(int(c) for c in cv2.applyColorMap(np.uint8([[int(255 * i / (len(BEAMS) - 1))]]), cv2.COLORMAP_TURBO)[0, 0]) for i in range(len(BEAMS))]
panels = []
for lab, o, Rt in ((f'from the poses.json origin', np.zeros(3), np.eye(3)), (f'from the solved LiDAR mount x {MNT[0]:+.3f} y {MNT[1]:+.3f} z {MNT[2]:+.3f} m, rx {MNT[3]:+.3f} ry {MNT[4]:+.3f} deg', np.array(MNT[:3]), R.from_euler('xy', MNT[3:5], degrees=True).as_matrix())):
    v = (q - o) @ Rt; r = np.linalg.norm(v, axis=1); k = r > 1.5; v, r = v[k], r[k]
    az = np.degrees(np.arctan2(v[:, 0], v[:, 1])); el = np.degrees(np.arcsin(v[:, 2] / r))
    im = np.zeros((Hh + 30, W, 3), np.uint8); px = ((az + 180) * SX).astype(int).clip(0, W - 1); py = ((E0 - el) * SY).astype(int)
    ok = (py >= 0) & (py < Hh); idx = np.argmin(np.abs(el[:, None] - np.array(BEAMS)[None]), 1); near = np.abs(el - np.array(BEAMS)[idx]) < TOL
    im[py[ok & ~near] + 30, px[ok & ~near]] = (90, 90, 90)
    for b in range(len(BEAMS)):
        m = ok & near & (idx == b); im[py[m] + 30, px[m]] = COL[b]
    for b, e in enumerate(BEAMS):
        y = int((E0 - e) * SY) + 30; cv2.line(im, (0, y), (14, y), COL[b], 1); cv2.putText(im, f'{e:+.1f}', (16, y + 4), 0, 0.35, COL[b], 1, cv2.LINE_AA)
    for a in range(-180, 181, 30): x = int((a + 180) * SX); cv2.line(im, (x, 26), (x, 30), (200, 200, 200), 1); cv2.putText(im, f'{a}', (x + 2, 24), 0, 0.35, (200, 200, 200), 1)
    cv2.putText(im, f'{s} t={f} {lab}: azimuth (0 = forward, + clockwise) x elevation, 6 px/deg x 20 px/deg; coloured = within {TOL} deg of a top-5 / bottom-10 design elevation', (60, 12), 0, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
    panels.append(im)
cv2.imwrite(OUT, np.vstack(panels)); print(OUT)
