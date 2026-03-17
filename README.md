# AvikuSim — 3D Gaussian Splatting Simulator

Simulation toolkit for 3D Gaussian Splatting (3DGS) scenes using NVIDIA Isaac Sim.

## Prerequisites

- NVIDIA Isaac Sim installed at `~/isaacsim`
- NVIDIA GPU with compatible driver (CUDA 12.x)

## Usage

Load a USD/USDZ scene with 6 boat-mounted fisheye cameras and live web view:

```bash
~/isaacsim/python.sh load_usd.py <path_to_usd_file>
```

Then open **http://localhost:8080** to see all 6 camera views.

### Options

| Flag | Description |
|------|-------------|
| `--headless` | Run without GUI (no rendering window) |
| `--calibration <json>` | Camera calibration file (default: `AquaSport2800_Calibration_UPDATE.json`) |
| `--port <port>` | Web server port (default: 8080) |

### Examples

```bash
# Load a 3DGS scene with GUI + camera web view
~/isaacsim/python.sh load_usd.py ../3dgs.usdz

# Headless mode (web view only)
~/isaacsim/python.sh load_usd.py ../3dgs.usdz --headless

# Custom port
~/isaacsim/python.sh load_usd.py ../3dgs.usdz --port 9090
```

## Boat & Camera Setup

The script creates a boat hull (default 9.5m x 3.0m) at `/World/Boat` and mounts 6 fisheye cameras using calibration extrinsics from the JSON file.

### Camera Layout

| # | Name | Position | Description |
|---|------|----------|-------------|
| 1 | Bow | Front center | Looking down (~92° pitch) |
| 2 | Starboard Bow | Front right | Looking forward-right (~41° pitch) |
| 3 | Port Bow | Front left | Looking forward-left (~39° pitch) |
| 4 | Starboard | Mid right | Looking down (~78° pitch) |
| 5 | Port | Mid left | Looking down (~78° pitch) |
| 6 | Stern | Rear center | Looking down (~85° pitch) |

### Fisheye Parameters

| Parameter | Value |
|-----------|-------|
| Resolution | 1280 x 720 |
| Optical center | (639.67, 361.30) px |
| Distortion model | f-theta (converted from OCamCalib) |

The original OCamCalib parameters (cam2world polynomial, inverse polynomial, affine correction) are stored as custom USD attributes on each camera prim under the `ocamcalib:` namespace.

### Web View

The MJPEG streaming server at `localhost:8080` provides:
- `/` — Dashboard with 3x2 grid of all camera views
- `/stream/<0-5>` — Individual camera MJPEG stream

## Environment

| Component | Version |
|-----------|---------|
| OS | Ubuntu 22.04.5 LTS |
| Kernel | 6.8.0-101-generic |
| GPU | NVIDIA GeForce RTX 4090 x4 |
| Driver | 575.57.08 |
| CUDA (runtime) | 12.9 |
| CUDA Toolkit | 12.4 |
| Isaac Sim | 5.1.0 |
| Python (Isaac Sim) | 3.11.13 |
| Python (system) | 3.10.12 |

## Project Structure

```
├── CLAUDE.md          # Development guidelines
├── README.md          # This file
├── load_usd.py        # Main script: USD loader + boat cameras + web server
├── AquaSport2800_Calibration_UPDATE.json  # Camera calibration data
└── docs/
    ├── CHANGELOG.md   # Development changelog
    └── planning/      # Planning documents
```

## Known Issues

- Isaac Sim's bundled `libnvJitLink.so.12` may need to be upgraded if you see `undefined symbol: __nvJitLinkCreate_12_8` errors. Replace the bundled library in `~/isaacsim/exts/omni.isaac.ml_archive/pip_prebundle/nvidia/nvjitlink/lib/` and `~/isaacsim/kit/python/lib/python3.11/site-packages/nvidia/nvjitlink/lib/` with a newer version (`pip install nvidia-nvjitlink-cu12`).
