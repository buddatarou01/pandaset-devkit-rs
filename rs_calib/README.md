# Rolling-shutter-aware LiDAR projection for PandaSet

The shipped PandaSet poses and intrinsics project the LiDAR well onto the images while the car stands still, but the
projection drifts as the car speeds up: the upper rows drift outward from the focus of expansion, the bottom rows stay in
place, and in turns the drift is sideways. All six cameras are rolling-shutter sensors (Leopard LI-USB30-AR023ZWDRB,
ON Semiconductor AR0231). The data are consistent with:

- each image timestamp is the moment the 360-degree LiDAR sweeps across the centre column of that camera
  (+0.4 to +0.7 ms in day, dusk and night scenes alike, so no exposure constant is added to it);
- the **bottom** image row is exposed at that timestamp (+ a small delta) and the rows above are exposed earlier, the top row
  about 30 ms earlier;
- the readout / delta differ somewhat between the day, dusk and night recordings (auto exposure);
- the five wide-angle cameras need small per-camera extrinsic corrections and a lens model with some distortion at the image
  edges (in `rs_params.json` all five use KB4, the front camera stays pinhole); `left_camera` and `right_camera` have the most.
  The distortion coefficients in `docs/static_extrinsic_calibration.yaml` do not fit the shipped images.

This folder holds the model, the parameters we tuned by eye, and a visualisation script. Nothing in the devkit itself is changed.

## Files

| file | content |
|---|---|
| `rs_model.py` | the projection model (`Projector`), `load_params`, `light_group` |
| `rs_params.json` | the parameters (see below) |
| `viz_projection.py` | side-by-side images / video: shipped projection vs. the model |
| `pandaset_io.py` | minimal reader for one PandaSet sequence (images, poses, timestamps, LiDAR) |

## Model

For an image of camera `c` with timestamp `t_img`, a LiDAR point is projected with the camera pose at the time of the row `v` it
lands on (3 fixed-point passes):

```
t(v)  = t_img + delta - readout * (1079 - v) / 1079            # ms; bottom row = t_img + delta
ego(t) = front-camera pose (poses.json) at t_img, moved with a constant velocity (central difference of the front poses)
pose_c = ego(t) * X_c * C_c                                     # X_c = shipped extrinsic (camera -> front camera, mean over frames)
                                                                # C_c = correction: shift (tx, ty, tz cm) + yaw / pitch / roll (deg) in the camera frame
pixel  = lens_c(K_c, point in camera c)                         # K_c = shipped fx, fy, cx + dcx, cy + dcy
                                                                # lens: pinhole, or KB4 theta_d = theta (1 + k1 th^2 + k2 th^4 + k3 th^6 + k4 th^8)
```

`readout` and `delta` are shared by all cameras of a light group. The group comes from the local (US Pacific) time of the
scene's first front-camera timestamp: 2019-05-11 morning = `day`, 2019-05-10 before 21:00 = `dusk`, 21:00 or later = `night`.

## Parameters (`rs_params.json`)

```json
{"timing": {"day":   {"readout": ms, "delta": ms}, "dusk": {...}, "night": {...}},
 "cameras": {"front_camera": {"yaw": deg, "pitch": deg, "roll": deg, "tx_cm": cm, "ty_cm": cm, "tz_cm": cm,
                              "dcx_px": px, "dcy_px": px, "lens": "pinhole" | "kb4", "k1": .., "k2": .., "k3": .., "k4": ..}, ...}}
```

Missing entries mean no correction and a pinhole lens. The values were set by eye on many scenes and frames and refined by a
least-squares fit to hand-picked objects; they are approximate. The shutter timing also jitters from frame to frame by roughly
1-2 ms, which no constant can remove; at speed this shows on near objects.

## Usage

```bash
pip install -e python                        # the devkit (for pandaset.geometry)
export PANDASET_ROOT=/path/to/pandaset        # one folder per sequence: 001, 002, ...
cd rs_calib
python viz_projection.py --scene 099 --cam front_right_camera --frames 0-79 --out out/099_front_right --video
```

```python
from pandaset_io import Scene
from rs_model import Projector, load_params
sc = Scene('099'); p = load_params('rs_params.json', sc, 'left_camera')
uv, depth, idx = Projector(sc, 'left_camera', p).project(sc.lidar(40, 0)[:, :3], 40)
```

## Licence and attribution

- The devkit code is Apache License 2.0 (see `LICENSE.md`); the files in this folder are additions to it.
- PandaSet data are provided by Scale AI and Hesai under CC BY 4.0 with additional Dataset Terms (pandaset.org). The parameters
  here are derived from PandaSet and are subject to the same terms. This work is not affiliated with or endorsed by Scale AI or
  Hesai.
