# rs_calib

Rolling-shutter-aware LiDAR projection for PandaSet. See the [repository README](../README.md) for what is wrong with the shipped
projection, the model, how the parameters were obtained, and usage.

| file | content |
|---|---|
| `rs_model.py` | projection model (`Projector`, `load_params`, `light_group`) |
| `rs_params.json` | parameters |
| `viz_projection.py` | images / video: shipped projection vs. model (`--layout vertical` for one above the other) |
| `pandaset_io.py` | minimal reader for one sequence (images, poses, timestamps, LiDAR); set `PANDASET_ROOT` |
