# PandaSet LiDAR–camera misalignment fixed: rolling shutter + time offset model for all 6 cameras

**If the PandaSet LiDAR does not line up with the camera images when you project it with the devkit** — points sliding off poles,
cars and building edges as the car speeds up, worse at the top of the image and in turns, worst on the side cameras — the cause is
not (only) the extrinsic calibration. The PandaSet cameras are **rolling-shutter** sensors, the dataset says nothing about when
each row was exposed, and the shipped projection treats every image as taken at one instant. This repository adds a small model that fixes it for all six cameras, with the parameters and a
visualisation script.

Keywords: PandaSet LiDAR camera misalignment, PandaSet rolling shutter, SplatAD PandaSet, PandaSet projection offset, PandaSet calibration wrong, pandaset-devkit projection,
PandaSet vs Waymo, different from Waymo,
rolling shutter, camera time offset, LiDAR camera synchronisation, extrinsic calibration, lens distortion, Pandar64.

![front_right_camera, scene 099 (about 19 m/s): top = shipped projection, bottom = rolling-shutter model](docs/rs_calib/099_front_right_t30-49.gif)

*Scene 099, `front_right_camera`, frames 30–49 at about 19 m/s, 3 frames/s. Top: shipped poses and intrinsics (devkit projection).
Bottom: this model. Colour = depth.*

## All six cameras, same frame (scene 003, frame 65, about 8.5 m/s, dusk)

Top half of each image: shipped projection. Bottom half: rolling-shutter model with `rs_calib/rs_params.json`.

**`front_camera`**

![front_camera](docs/rs_calib/003_front_camera_t65.jpg)

**`front_left_camera`**

![front_left_camera](docs/rs_calib/003_front_left_camera_t65.jpg)

**`front_right_camera`**

![front_right_camera](docs/rs_calib/003_front_right_camera_t65.jpg)

**`left_camera`**

![left_camera](docs/rs_calib/003_left_camera_t65.jpg)

**`right_camera`**

![right_camera](docs/rs_calib/003_right_camera_t65.jpg)

**`back_camera`**

![back_camera](docs/rs_calib/003_back_camera_t65.jpg)

## Zoom on poles, one per camera

Crops around a pole (or a thin post) for each camera, at the fastest frame where we marked one. Left: shipped projection. Right:
this model. Scene, frame and speed are written above each pair.

![poles, shipped vs model, all six cameras](docs/rs_calib/pole_zoom.jpg)

Make the same pictures (and a video) for any scene and camera:

```bash
pip install -e python                         # the devkit (pandaset.geometry)
export PANDASET_ROOT=/path/to/pandaset         # one folder per sequence: 001, 002, ...
cd rs_calib
python viz_projection.py --scene 099 --cam front_right_camera --frames 0-79 --out out/099_front_right --video --layout vertical
```

## What is wrong with the shipped projection

What is actually known about the cameras is little:

- **They are rolling-shutter sensors that read out top to bottom.** That much follows from the camera model in the paper
  (Leopard LI-USB30-AR023ZWDRB, ON Semiconductor AR0231, electronic rolling shutter).
- **Nothing tells you when the rows were exposed.** PandaSet ships one timestamp per image and no shutter information: no readout
  start / end, no exposure time, no trigger time. (Other datasets ship these, e.g. Waymo Open gives `camera_trigger_time`,
  `camera_readout_done_time` and `shutter` for every image.) The devkit treats every row as taken at the timestamp.
- **The timestamp itself cannot be trusted.** The paper describes it as the trigger time (the 360-degree LiDAR crossing the centre of
  the camera's field of view) plus an estimated exposure time, one constant for day and one for night. In the data it equals the
  LiDAR crossing of the centre column (+0.4 to +0.7 ms) in day, dusk and night scenes alike, so no exposure constant shows up, and
  the projection does not fit at speed. Which instant the timestamp stands for, and how long the readout takes, has to be
  recovered by aligning the images with the LiDAR.
- **The field of view does not match the paper either.** The paper gives 107 deg horizontal for the five wide-angle cameras; the
  shipped `intrinsics.json` (fx about 930 px on 1920 px, pinhole, no distortion) gives about 92 deg. The images are distorted at the
  edges, and the distortion coefficients in `docs/static_extrinsic_calibration.yaml` do not fit the shipped images, so the lens
  model has to be recovered as well.
- While the car stands still, the shipped projection fits well and the error grows with speed. This is why it is easy to blame the
  extrinsics, and why re-calibrating the extrinsics on moving data gives a different, speed-dependent answer.

What we recovered by aligning with the LiDAR (the model below, values in `rs_calib/rs_params.json`): with the rows read out top to
bottom, the projection fits when the bottom row is taken 3–14 ms after the timestamp (day / dusk / night) and the top row about
30 ms before the bottom row; at 19 m/s that is about 0.6 m of car motion between the top and the bottom of one image. The readout and the offset differ between the
day, dusk and night recordings (auto exposure), the shutter timing jitters by roughly 1–2 ms from frame to frame, and the
wide-angle cameras need small extrinsic corrections and a KB4 lens. This is a fit, not a specification; nothing in the dataset
states it.

**The individual values are not necessarily the physical ones.** The timing is a parameter of the model, but depending on the
camera and on how the car moves it is partly degenerate with the camera position and the principal point: for a side camera
while driving straight, a time offset, a shift of the camera along the driving direction and a shift of cx all move the projection
in nearly the same way, and a longer readout can be traded against a later offset. The fit picks one combination of these. What
is checked is the result: the LiDAR lines up with the images (pole, car and building edges, across speeds, rows and cameras). Do
not read `readout`, `delta`, `tx_cm` ... as measured sensor properties.

## Different from Waymo: what PandaSet does not ship

Waymo Open Dataset uses rolling-shutter cameras too, but ships the timing and the motion needed to project correctly. PandaSet does not:

| per image / per camera | Waymo Open Dataset (v2) | PandaSet |
|---|---|---|
| trigger time | `camera_trigger_time` | no |
| end of readout | `camera_readout_done_time` | no |
| exposure time | `shutter` | no (the paper mentions one estimated constant for day and one for night) |
| readout direction | `rolling_shutter_direction` per camera | no (top to bottom, from the sensor model) |
| ego pose and velocity at the image | `pose`, `velocity` per image | camera pose per image, no velocity |
| lens distortion | k1, k2, p1, p2, k3 per camera | none in `intrinsics.json`; the YAML coefficients do not fit the images |
| LiDAR point -> pixel | `camera_projection` per LiDAR point | no |
| image timestamp | consistent with the above | one value; what it stands for has to be recovered from the LiDAR |

So with PandaSet the shutter timing, the readout and the lens have to be estimated from the data before the LiDAR can be
projected onto moving images. This repository does that.

## The model (`rs_calib/rs_model.py`)

For an image of camera `c` with timestamp `t_img`, every LiDAR point is projected with the camera pose at the time of the image row
`v` it lands on (3 fixed-point passes):

```
t(v)   = t_img + delta - readout * (1079 - v) / 1079     # ms; bottom row = t_img + delta, top row readout earlier
ego(t) = front-camera pose (poses.json) at t_img, moved with constant velocity (central difference of the front poses)
pose_c = ego(t) * X_c * C_c     # X_c: shipped extrinsic (camera -> front camera, mean over frames)
                                # C_c: correction, shift tx, ty, tz (cm) + yaw / pitch / roll (deg) in the camera frame
pixel  = lens_c(K_c, point)     # K_c: shipped fx, fy, cx + dcx, cy + dcy; lens pinhole or Kannala-Brandt 4 terms (KB4)
```

`readout` and `delta` are shared by all cameras of one light group, chosen from the local (US Pacific) time of the scene's first
timestamp: 2019-05-11 morning = `day`, 2019-05-10 before 21:00 = `dusk`, 21:00 or later = `night`.

```python
import sys; sys.path.insert(0, 'rs_calib')
from pandaset_io import Scene
from rs_model import Projector, load_params

sc = Scene('099')                                            # PANDASET_ROOT must be set
p = load_params('rs_calib/rs_params.json', sc, 'left_camera')
uv, depth, idx = Projector(sc, 'left_camera', p).project(sc.lidar(40, 0)[:, :3], 40)   # pixels, depth, index of the drawn points
```

## Why this approach

Without the shutter data that Waymo ships, PandaSet is hard to use for research that needs the LiDAR and the images to agree on
moving frames (depth supervision, LiDAR–camera fusion, learned calibration, neural rendering).

As far as we know, SplatAD (Gaussian splatting for autonomous driving) handles PandaSet by modelling the rolling shutter **and**
optimising the camera poses at the same time. Then a timing error can be absorbed by the poses, and the result is a camera pose
that changes with the speed of the car rather than a fixed rig.

To keep the extrinsics fixed and still project the LiDAR correctly, the shutter timing has to be a variable of its own, solved
from correspondences between the LiDAR and the images that are known to be right. Two obvious ways of getting there did not work
for us:

- **Tuning all cameras by hand at once is whack-a-mole.** The timing is shared by the cameras and partly degenerate with each
  camera's position and principal point, so fixing one camera or one frame breaks another.
- **Automatic edge alignment between the LiDAR and the image is unreliable here.** LiDAR flare / blooming on bright and reflective
  surfaces, occlusion and the parallax between the LiDAR and each camera put LiDAR edges where the image has none, and the
  alignment follows them.

So the correspondences are made by hand but kept small and rough: a few objects in the foreground are picked by hand (poles,
signs, cars, building corners), segmented, and each segment is matched to the LiDAR cluster that best fits its shape. **That the
objects are chosen by hand is the point:** a person decides that this segment and this cluster are the same thing, which no edge
or flare can fool, and the solver below does the rest for all cameras and sequences together.

## How the parameters were obtained

The parameters were not fitted to a full ground truth. They come from a small number of object-level correspondences between
image segments and LiDAR points, collected interactively, and one solver shared by all sequences and cameras.

1. **Start from the shipped calibration** and look at the projection frame by frame (the side-by-side view above). Coarse values
   (readout, delta, a few tenths of a degree, a few cm, the KB4 coefficients) were first set by eye on frames where the drift is
   obvious: high speed, near poles and cars, turns, the image edges.
2. **Object correspondences ("snapshots").** On a frame, click an object (pole, sign, car, building corner). A SAM 3 point prompt
   segments it. The LiDAR points that project into the mask with the current parameters are split into depth groups (a new group
   starts at a depth gap larger than max(0.5 m, 5 %)). The object's group is the one with the largest
   (points inside the mask) x (share of the LiDAR points at that depth, inside a box 1.5x the mask, that fall in the mask): the
   object lies mostly inside its mask, the background behind it spreads outside, stray points are few. The group's points
   (world coordinates) and the mask are stored. This takes a second per object and needs no annotation.
3. **One solver for everything.** All snapshots of all sequences and cameras go into one robust least-squares problem (a small
   factor graph):
   - variables: `readout`, `delta` per light group (shared by all cameras); per camera the extrinsic correction (yaw, pitch, roll,
     tx, ty, tz), the principal-point shift (dcx, dcy) and, for KB4 cameras, k1..k4;
   - one factor per snapshot point: the px distance of its projection (with the full model above) to the snapshot's mask
     (0 inside, bilinear distance transform outside; or, alternatively, to the pixel the point had when the snapshot was taken);
     10 points per object, spread from top to bottom so that every object weighs the same and the rows constrain the readout;
   - one prior per variable that keeps it near its current value (0.2 deg, 5 cm, 10 px, 5 ms, 0.05 for k), so variables that no
     snapshot constrains do not drift;
   - Huber loss (1 px), trust-region least squares with a forward-difference Jacobian in which each variable re-projects only the
     snapshots it affects (a camera's variables only that camera's points, a light group's timing only that group's scenes).
4. **Iterate.** Solve a few iterations, look at the projection again, add snapshots where it still drifts (other scenes, speeds,
   image rows, cameras), solve again. 199 objects in 23 scenes were used (84 day, 115 dusk; front 13, front_left 45,
   front_right 36, left 35, right 58, back 12). The night timing has no snapshots and was set by eye only.

The tool used for steps 1–4 (an interactive web page with the snapshot and the solve) is not part of this repository; the model,
the parameters and the visualisation here are enough to use the result.

## Parameters (`rs_calib/rs_params.json`)

```json
{"timing":  {"day": {"readout": ms, "delta": ms}, "dusk": {...}, "night": {...}},
 "cameras": {"left_camera": {"yaw": deg, "pitch": deg, "roll": deg, "tx_cm": cm, "ty_cm": cm, "tz_cm": cm,
                             "dcx_px": px, "dcy_px": px, "lens": "pinhole" | "kb4", "k1": .., "k2": .., "k3": .., "k4": ..}, ...}}
```

The values were set by eye on many scenes and refined with a least-squares fit to hand-picked objects. They reproduce the
LiDAR alignment; individually they may not be the physical values (timing, camera position and principal point are partly
degenerate, see above). They are approximate:
the shutter timing also jitters by roughly 1–2 ms from frame to frame, which no constant removes; at speed this still shows on near
objects. Missing entries mean no correction and a pinhole lens.

## Alternative: solved with inter-camera overlap (`rs_calib/rs_params_overlap.json`)

Same model, same format, one more kind of constraint in the solve: neighbouring cameras must agree with each other where their
views overlap.

- **Pairs.** front–front_right, front_right–right, left–front_left, front_left–front, back–left, on frames 10 and 50 of 8 scenes
  (4 day: 158, 099, 080, 124; 4 dusk: 001, 013, 008, 003). right–back has no LiDAR point seen by both cameras on any of these
  frames, so the ring is not closed at the back.
- **Points.** Static LiDAR points of that frame seen by both cameras A and B: points inside cuboids that are not stationary
  (moving cars, pedestrians) are removed, since the two cameras expose at different times; only the nearest depth in each 5 px
  cell is kept in both views.
- **Factor.** RoMa v2 (dense matcher) gives the pixel in B of each pixel in A. The point projected into B with the full model
  (rolling shutter: the camera pose at the time of the row it lands on in B) must land where RoMa carries its projection in A
  (with the pose at the time of its row in A). The factor touches both cameras' variables and the light group's timing, i.e. it
  constrains the pose of each row in both images. 30 points per pair, spread over the rows, RoMa certainty > 0.9.
- These factors are added to the object snapshots above and solved together.

Measured against `rs_params.json` (all points of every snapshot; overlap points with RoMa certainty > 0.9; px, median over pairs
or snapshots of the per-pair / per-snapshot median):

| | `rs_params.json` | `rs_params_overlap.json` |
|---|---|---|
| overlap front_left → front (7 pairs; worst pair) | 4.29 (5.49) | **6.73 (10.58)** |
| overlap front → front_right (14 pairs; worst pair) | 3.65 (6.78) | 1.63 (2.67) |
| overlap front_right → right (16 pairs; worst pair) | 3.69 (5.82) | 1.50 (2.40) |
| overlap left → front_left (16 pairs; worst pair) | 4.21 (5.78) | 1.27 (5.35) |
| overlap back → left (2 pairs; worst pair) | 2.73 (4.05) | 2.69 (3.58) |
| LiDAR vs object snapshots, 205 objects (p90) | 1.51 (4.74) | **2.54 (5.50)** |

With the overlap the object snapshots fit worse (111 of 205 by more than 0.5 px, 24 better; back camera unchanged at 0.70 →
0.69), and front_left → front gets worse. Even RoMa's most certain matches (> 0.99) disagreed with the projection by about 3.6 px
before this solve, a systematic offset, so part of what the overlap term fits may be RoMa's own bias rather than the calibration.
Timing in this file: day readout 25.12 ms / delta 1.65 ms, dusk 29.11 ms / 4.92 ms (night unchanged, by eye).

![scene 158 frame 10, front_right_camera: red = LiDAR projected into front_right, green = RoMa of its projection in front_camera; top rs_params.json, bottom rs_params_overlap.json](docs/rs_calib/overlap_158_front_to_front_right.jpg)

*Scene 158, frame 10, `front_right_camera`, the overlap with `front_camera`. Red: static LiDAR points projected into
front_right. Green: where RoMa v2 carries their projection in front_camera. Top: `rs_params.json`. Bottom:
`rs_params_overlap.json`. The median / p90 / max of |red − green| are written on each half.*

    python viz_projection.py --scene 099 --cam front_left_camera --frames 0-79 --params rs_params_overlap.json --out out/099_fl --video

## Files

| file | content |
|---|---|
| `rs_calib/rs_model.py` | projection model (`Projector`, `load_params`, `light_group`) |
| `rs_calib/rs_params.json` | parameters |
| `rs_calib/rs_params_overlap.json` | parameters solved with the inter-camera overlap term (see above) |
| `rs_calib/viz_projection.py` | images / video: shipped projection vs. model, side by side or one above the other |
| `rs_calib/pandaset_io.py` | minimal reader for one sequence (images, poses, timestamps, LiDAR) |
| `README_devkit.md`, `python/`, `tutorials/` | the original pandaset-devkit, unchanged |

## Licence and attribution

- The devkit is Apache License 2.0 (`LICENSE.md`, Copyright Scale AI); the `rs_calib/` files and `docs/rs_calib/` images are additions.
- PandaSet is provided by Scale AI and Hesai under CC BY 4.0 with additional Dataset Terms (pandaset.org). The parameters and the
  images here are derived from PandaSet and are subject to the same terms. This work is not affiliated with or endorsed by Scale AI or
  Hesai.

## Note

Datasets such as Waymo Open and nuScenes are well prepared, but their LiDAR data do not reach far (Waymo Open: the top LiDAR
returns are cut at 75 m; nuScenes: a 32-beam LiDAR). PandaSet is one of the few datasets with LiDAR returns well beyond 150 m
(Pandar64 rated 200 m at 10 % reflectivity, the forward PandarGT 300 m) and cameras covering 360 degrees, which makes it valuable
for long-range work. Its devkit, however, does not give a correct LiDAR–camera correspondence on moving frames, so this
repository is published to provide one.

If you would like to know more about the tools and the method used to obtain the parameters, please open an issue in this
repository.

