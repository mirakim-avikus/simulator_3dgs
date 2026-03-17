# Changelog

## 2026-03-17
- Added 6-camera boat setup using `AquaSport2800_Calibration_UPDATE.json`
  - Creates boat hull (9.5m x 3.0m) at `/World/Boat` with biased rotation trim correction
  - Mounts 6 fisheye cameras: bow, starboard bow, port bow, starboard, port, stern
  - Each camera uses OCamCalib f-theta distortion model with calibrated extrinsics
  - Camera positions/rotations loaded from calibration JSON (boat frame: X-forward, Y-starboard, Z-up)
- Added MJPEG web server on `localhost:8080`
  - 3x2 grid view of all 6 cameras with live streaming
  - Endpoints: `/` (dashboard), `/stream/<0-5>` (per-camera MJPEG)
  - `ThreadingHTTPServer` for concurrent stream connections
- Camera capture via Kit viewport windows + `capture_viewport_to_buffer`
  - Bypasses `omni.syntheticdata` OmniGraph pipeline to avoid MGPU segfault during GUI interaction
  - Uses `omni.renderer_capture` (Hydra native) instead of replicator render products
  - Offscreen viewports (position=-200, 2x2px) — not visible in GUI
  - Double-buffered batch capture: all 6 frames flush atomically for synchronized web display
  - Trade-off: sequential GPU readback (~2-3 fps) vs parallel but crash-prone sensor approach
- New CLI options: `--calibration <json>`, `--port <port>`

## 2026-03-16
- Added `load_usd.py`: USD file loader script using Isaac Sim (`~/isaacsim`)
  - Supports local file paths and Nucleus URLs
  - Prints stage hierarchy on load
  - Adds default lighting if none exists
  - Runs interactive simulation loop with GUI (or headless mode)
  - Added fisheye camera with OCamCalib parameters (Scaramuzza model)
    - Converts OCamCalib cam2world polynomial to Isaac Sim f-theta distortion model
    - Stores original OCamCalib parameters as custom USD attributes
  - Fixed libnvJitLink.so.12 version mismatch (upgraded bundled 12.3 → 12.9)
  - Removed torch-dependent imports to avoid cusparse/nvjitlink load failures
