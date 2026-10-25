"""LiDAR projection side by side: left = shipped poses / intrinsics (devkit projection), right = rolling-shutter model with
rs_params.json. One image per frame, optionally an H.264 video (needs ffmpeg).

    PANDASET_ROOT=/path/to/pandaset python viz_projection.py --scene 099 --cam front_right_camera --frames 0-79 --out out/099_fr --video
"""
import argparse
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cv2
import numpy as np

from pandaset_io import CAMS, Scene, inv
from rs_model import Projector, light_group, load_params


def overlay(img_rgb, uv, z, title, zmax=30.0, dot=2):
    out = (cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR) * 0.6).astype(np.uint8)
    col = cv2.applyColorMap(np.clip(z / zmax * 255, 0, 255).astype(np.uint8)[:, None], cv2.COLORMAP_JET)[:, 0]
    for (u, v), c in zip(uv.astype(int), col):
        cv2.circle(out, (int(u), int(v)), dot, c.tolist(), -1)
    cv2.putText(out, title, (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 255), 3)
    return out


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--scene', required=True); ap.add_argument('--cam', default='front_camera', choices=CAMS)
    ap.add_argument('--frames', default='0-79', help='a-b or comma list'); ap.add_argument('--params', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rs_params.json'))
    ap.add_argument('--out', required=True); ap.add_argument('--video', action='store_true'); ap.add_argument('--fps', type=int, default=10)
    a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
    fr = range(int(a.frames.split('-')[0]), int(a.frames.split('-')[1]) + 1) if '-' in a.frames else [int(x) for x in a.frames.split(',')]
    sc = Scene(a.scene); p = load_params(a.params, sc, a.cam); pr = Projector(sc, a.cam, p); K0 = sc.K(a.cam); g = light_group(sc)
    for i, t in enumerate(fr):
        img = sc.image(a.cam, t); P = sc.lidar(t, 0)[:, :3]
        Tcw = inv(sc.T_wc(a.cam, t)); q = P @ Tcw[:3, :3].T + Tcw[:3, 3]; m = q[:, 2] > 0.3; uv0 = (q[m] @ K0.T)[:, :2] / q[m, 2:3]
        k = (uv0[:, 0] >= 0) & (uv0[:, 0] < 1920) & (uv0[:, 1] >= 0) & (uv0[:, 1] < 1080)
        Lft = overlay(img, uv0[k], q[m][k, 2], f'{a.scene} {a.cam} t={t}  shipped (poses.json + intrinsics.json)')
        uv, z, _ = pr.project(P, t)
        Rgt = overlay(img, uv, z, f'{a.scene} t={t}  rolling shutter {g}: readout {p["readout"]:.1f} ms delta {p["delta"]:+.1f} ms, {p["lens"]}')
        frame = cv2.resize(np.hstack([Lft, np.full((1080, 12, 3), 255, np.uint8), Rgt]), (1926, 540))
        cv2.imwrite(f'{a.out}/{i:03d}.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
    print(f'{len(fr)} frames -> {a.out}')
    if a.video:
        mp4 = a.out.rstrip('/') + '.mp4'
        subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate', str(a.fps), '-i', f'{a.out}/%03d.jpg', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                        '-crf', '20', '-vf', 'pad=ceil(iw/2)*2:ceil(ih/2)*2', mp4], check=True)
        print(mp4)


if __name__ == '__main__':
    main()
