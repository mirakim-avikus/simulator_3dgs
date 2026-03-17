"""Load a USD file, create a boat with 6 fisheye cameras, and serve views on localhost:8080."""

import argparse
import io
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Load a USD file in Isaac Sim with boat cameras")
parser.add_argument("usd_path", help="Path to .usd/.usda/.usdz file (local or Nucleus URL)")
parser.add_argument("--headless", action="store_true", help="Run without GUI")
parser.add_argument("--calibration", default=None,
                    help="Camera calibration JSON (default: AquaSport2800_Calibration_UPDATE.json)")
parser.add_argument("--port", type=int, default=8080, help="Web server port")
args = parser.parse_args()

usd_path = args.usd_path
if not usd_path.startswith("omniverse://") and not os.path.isabs(usd_path):
    usd_path = os.path.abspath(usd_path)

calib_path = args.calibration or os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "AquaSport2800_Calibration_UPDATE.json")

simulation_app = SimulationApp({"headless": args.headless})

import ctypes

import omni.usd
from omni.kit.viewport.utility import create_viewport_window, capture_viewport_to_buffer
from pxr import Gf, Sdf, UsdGeom, UsdLux

try:
    from PIL import Image

    def encode_jpeg(rgb_array):
        buf = io.BytesIO()
        Image.fromarray(rgb_array).save(buf, format="JPEG", quality=80)
        return buf.getvalue()
except ImportError:
    import cv2

    def encode_jpeg(rgb_array):
        _, buf = cv2.imencode(".jpg", cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR),
                              [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes()

# --------------------------------------------------------------------------
# OCamCalib fisheye parameters
# --------------------------------------------------------------------------
OCAM_SS = [-2.907366e+02, 0.0, 6.915871e-04, 5.741715e-08, 3.065713e-10]
OCAM_INVPOL = [590.937502, 522.678139, 114.990712, 23.814065,
               45.831575, 49.789384, 20.772449, 3.232227]
OCAM_CENTER_ROW, OCAM_CENTER_COL = 361.303803, 639.669324
OCAM_AFFINE_C, OCAM_AFFINE_D, OCAM_AFFINE_E = 1.001320, -0.001614, 0.001861
OCAM_HEIGHT, OCAM_WIDTH = 720, 1280

CAMERA_NAMES = ["bow", "starboard_bow", "port_bow", "starboard", "port", "stern"]
STREAM_WIDTH, STREAM_HEIGHT = 640, 360

latest_frames = [None] * 6
frame_lock = threading.Lock()


def ocam_to_ftheta_coeffs(ss, center_row, center_col, height, width, num_samples=500):
    max_r = math.sqrt(max(center_col, width - center_col) ** 2 +
                      max(center_row, height - center_row) ** 2)
    rho = np.linspace(0, max_r, num_samples)
    z = sum(c * rho ** i for i, c in enumerate(ss))
    theta = np.arctan2(rho, -z)
    coeffs = np.polyfit(rho, theta, deg=4)[::-1].tolist()
    max_fov = float(np.degrees(theta[-1])) * 2
    return coeffs, max_fov


def compute_camera_orient(pitch_deg, roll_deg, yaw_deg):
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    roll = math.radians(roll_deg)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    forward = Gf.Vec3d(cp * cy, -cp * sy, -sp)
    if forward.GetLength() < 1e-9:
        forward = Gf.Vec3d(0, 0, -1)
    forward = forward.GetNormalized()
    cam_z = -forward

    world_up = Gf.Vec3d(0, 0, 1)
    cam_x = (world_up ^ cam_z)
    if cam_x.GetLength() < 1e-6:
        fallback = Gf.Vec3d(cy, -sy, 0)
        if fallback.GetLength() < 1e-6:
            fallback = Gf.Vec3d(1, 0, 0)
        cam_x = (fallback.GetNormalized() ^ cam_z)
    cam_x = cam_x.GetNormalized()
    cam_y = (cam_z ^ cam_x).GetNormalized()

    cr, sr = math.cos(roll), math.sin(roll)
    cam_x_r = cam_x * cr + cam_y * sr
    cam_y_r = cam_y * cr - cam_x * sr

    rot_mat = Gf.Matrix4d(1.0)
    rot_mat.SetRow3(0, cam_x_r)
    rot_mat.SetRow3(1, cam_y_r)
    rot_mat.SetRow3(2, cam_z)
    rotation = rot_mat.ExtractRotation()
    quat = rotation.GetQuat()
    return Gf.Quatd(quat.GetReal(), *quat.GetImaginary())


def create_fisheye_camera(stage, prim_path, position, pitch, roll, yaw):
    ftheta_coeffs, max_fov = ocam_to_ftheta_coeffs(
        OCAM_SS, OCAM_CENTER_ROW, OCAM_CENTER_COL, OCAM_HEIGHT, OCAM_WIDTH)
    camera = UsdGeom.Camera.Define(stage, Sdf.Path(prim_path))
    xform = UsdGeom.Xformable(camera.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    orient_d = compute_camera_orient(pitch, roll, yaw)
    orient_f = Gf.Quatf(orient_d)
    xform.AddOrientOp().Set(orient_f)

    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10000.0))
    camera.GetFocalLengthAttr().Set(2.0)
    camera.GetHorizontalApertureAttr().Set(4.0)
    camera.GetVerticalApertureAttr().Set(4.0 * OCAM_HEIGHT / OCAM_WIDTH)

    prim = camera.GetPrim()
    prim.ApplyAPI("OmniLensDistortionFthetaAPI")
    prim.GetAttribute("omni:lensdistortion:model").Set("ftheta")
    prim.GetAttribute("omni:lensdistortion:ftheta:opticalCenter").Set(
        Gf.Vec2f(OCAM_CENTER_COL, OCAM_CENTER_ROW))
    prim.GetAttribute("omni:lensdistortion:ftheta:nominalWidth").Set(float(OCAM_WIDTH))
    prim.GetAttribute("omni:lensdistortion:ftheta:nominalHeight").Set(float(OCAM_HEIGHT))
    prim.GetAttribute("omni:lensdistortion:ftheta:maxFov").Set(max_fov)
    for i, c in enumerate(ftheta_coeffs):
        prim.GetAttribute(f"omni:lensdistortion:ftheta:k{i}").Set(float(c))

    ns = "ocamcalib"
    prim.CreateAttribute(f"{ns}:ss", Sdf.ValueTypeNames.FloatArray, False).Set(OCAM_SS)
    prim.CreateAttribute(f"{ns}:invpol", Sdf.ValueTypeNames.FloatArray, False).Set(OCAM_INVPOL)
    prim.CreateAttribute(f"{ns}:center", Sdf.ValueTypeNames.Float2, False).Set(
        Gf.Vec2f(OCAM_CENTER_ROW, OCAM_CENTER_COL))
    prim.CreateAttribute(f"{ns}:affine", Sdf.ValueTypeNames.Float3, False).Set(
        Gf.Vec3f(OCAM_AFFINE_C, OCAM_AFFINE_D, OCAM_AFFINE_E))
    prim.CreateAttribute(f"{ns}:imageSize", Sdf.ValueTypeNames.Int2, False).Set(
        Gf.Vec2i(OCAM_HEIGHT, OCAM_WIDTH))
    return camera


# --------------------------------------------------------------------------
# Web server
# --------------------------------------------------------------------------
class StreamHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *a):
        pass

    def do_GET(self):
        if self.path == "/":
            self._serve_index()
        elif self.path.startswith("/stream/"):
            try:
                idx = int(self.path.split("/")[-1])
                if 0 <= idx < 6:
                    self._stream(idx)
                    return
            except ValueError:
                pass
            self.send_error(404)
        else:
            self.send_error(404)

    def _serve_index(self):
        html = """<!DOCTYPE html><html><head><title>AvikuSim Cameras</title>
<style>
*{box-sizing:border-box}
body{margin:0;background:#111;color:#eee;font-family:monospace}
h1{text-align:center;padding:10px 0 4px;margin:0;font-size:17px}
.grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:4px;padding:6px}
.cam{position:relative;background:#222;aspect-ratio:1280/720}
.cam img{width:100%;height:100%;object-fit:contain;display:block}
.label{position:absolute;top:4px;left:4px;background:rgba(0,0,0,.75);
  padding:2px 8px;font-size:12px;border-radius:3px}
</style></head><body>
<h1>AvikuSim &mdash; 6-Camera Boat View</h1>
<div class="grid">
"""
        for i, name in enumerate(CAMERA_NAMES):
            label = f"CAM{i+1} {name.replace('_', ' ').upper()}"
            html += (f'<div class="cam"><img src="/stream/{i}" alt="{name}">'
                     f'<div class="label">{label}</div></div>\n')
        html += "</div></body></html>"
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.end_headers()
        self.wfile.write(html.encode())

    def _stream(self, idx):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        prev_id = None
        try:
            while True:
                with frame_lock:
                    data = latest_frames[idx]
                cur_id = id(data)
                if data is not None and cur_id != prev_id:
                    prev_id = cur_id
                    hdr = (f"--frame\r\nContent-Type: image/jpeg\r\n"
                           f"Content-Length: {len(data)}\r\n\r\n").encode()
                    self.wfile.write(hdr)
                    self.wfile.write(data)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                time.sleep(0.033)
        except (BrokenPipeError, ConnectionResetError):
            pass


def start_web_server(port):
    ThreadingHTTPServer.allow_reuse_address = True
    server = ThreadingHTTPServer(("0.0.0.0", port), StreamHandler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"  Web server: http://localhost:{port}")


# ==========================================================================
# Main
# ==========================================================================
with open(calib_path) as f:
    calib = json.load(f)

boat_length = calib["boat_type"]["custom_length"]
boat_breadth = calib["boat_type"]["custom_breadth"]
bias = calib["biased_rotation_angle"]
cam_configs = calib["extrinsic_parameter"]["cameras"]
z_offset = calib["extrinsic_parameter"]["camera_z_pose_offset"]

ctx = omni.usd.get_context()
if not ctx.open_stage(usd_path):
    print(f"Failed to open: {usd_path}")
    simulation_app.close()
    sys.exit(1)

print(f"Loaded: {usd_path}")
stage = ctx.get_stage()

print("\n--- Stage Hierarchy ---")
for prim in stage.Traverse():
    print(f"  {prim.GetPath()} ({prim.GetTypeName()})")

if not [p for p in stage.Traverse() if p.IsA(UsdLux.DistantLight) or p.IsA(UsdLux.DomeLight)]:
    UsdLux.DistantLight.Define(stage, Sdf.Path("/DefaultLight")).CreateIntensityAttr(500)
    print("Added default distant light")

# --------------------------------------------------------------------------
# Boat
# --------------------------------------------------------------------------
print(f"\nBoat: {boat_length}m x {boat_breadth}m")
boat_xform = UsdGeom.Xform.Define(stage, Sdf.Path("/World/Boat"))
bx = UsdGeom.Xformable(boat_xform.GetPrim())
bx.ClearXformOpOrder()
bx.AddRotateXOp().Set(float(bias["roll"]))
bx.AddRotateYOp().Set(float(bias["pitch"]))
bx.AddRotateZOp().Set(float(bias["yaw"]))

hull = UsdGeom.Cube.Define(stage, Sdf.Path("/World/Boat/Hull"))
hx = UsdGeom.Xformable(hull.GetPrim())
hx.ClearXformOpOrder()
hx.AddScaleOp().Set(Gf.Vec3d(boat_length / 2, boat_breadth / 2, 0.15))

# --------------------------------------------------------------------------
# 6 Fisheye cameras
# --------------------------------------------------------------------------
print("\nCameras:")
camera_paths = []
for i, cc in enumerate(cam_configs):
    name = CAMERA_NAMES[i]
    loc = cc["location"]
    rot = cc["rotation"]
    pos = (loc["translation_x"], loc["translation_y"], -(loc["translation_z"] + z_offset))
    path = f"/World/Boat/Cam_{name}"
    camera_paths.append(path)
    create_fisheye_camera(stage, path, pos, rot["pitch"], rot["roll"], rot["yaw"])
    print(f"  [{i+1}] {name:15s}  pos=({pos[0]:+6.2f}, {pos[1]:+5.2f}, {pos[2]:+5.2f})  "
          f"P={rot['pitch']:+6.1f}° R={rot['roll']:+5.1f}° Y={rot['yaw']:+5.1f}°")

# --------------------------------------------------------------------------
# Viewport windows — one per camera.
# Uses Kit's native viewport rendering + omni.renderer_capture for frame
# readback. This bypasses omni.syntheticdata entirely, avoiding the MGPU
# OmniGraph crash.
# --------------------------------------------------------------------------
print("\nSetting up viewport windows...")
for _ in range(30):
    simulation_app.update()

viewports = []
for i, cam_path in enumerate(camera_paths):
    name = CAMERA_NAMES[i]
    vp = create_viewport_window(
        f"Cam_{name}",
        width=2, height=2,
        position_x=-200, position_y=-200,
        camera_path=Sdf.Path(cam_path),
    )
    if vp is None:
        print(f"  [!] Failed to create viewport for {name}")
        continue
    vp.viewport_api.resolution = (STREAM_WIDTH, STREAM_HEIGHT)
    viewports.append((i, vp))
    print(f"  [{i+1}] Viewport: Cam_{name} -> {cam_path}")
    for _ in range(5):
        simulation_app.update()

# Warm up all viewports
for _ in range(30):
    simulation_app.update()

print(f"  {len(viewports)} viewports ready")

# Pre-configure PyCapsule extraction
ctypes.pythonapi.PyCapsule_GetPointer.restype = ctypes.c_void_p
ctypes.pythonapi.PyCapsule_GetPointer.argtypes = [ctypes.py_object, ctypes.c_char_p]


# Double-buffer: staging collects a batch, then flushes to latest_frames atomically
_staging = [None] * 6
_staging_count = [0]
_batch_ready = [False]


def make_capture_cb(idx):
    """Create a capture callback for camera idx."""
    def on_captured(buffer, buffer_size, width, height, byte_format):
        try:
            if buffer_size <= 0 or width <= 0 or height <= 0:
                return
            channels = buffer_size // (width * height)
            if channels < 3:
                return
            ptr = ctypes.pythonapi.PyCapsule_GetPointer(buffer, None)
            arr = np.ctypeslib.as_array(
                (ctypes.c_uint8 * buffer_size).from_address(ptr)).copy()
            img = arr.reshape(height, width, channels)
            jpeg = encode_jpeg(img[:, :, :3])
            _staging[idx] = jpeg
            _staging_count[0] += 1
            # When all 6 are captured, flush atomically
            if _staging_count[0] >= len(viewports):
                with frame_lock:
                    for j in range(6):
                        if _staging[j] is not None:
                            latest_frames[j] = _staging[j]
                _batch_ready[0] = True
        except Exception:
            pass
    return on_captured


start_web_server(args.port)

# --------------------------------------------------------------------------
# Simulation loop — one batch at a time, all 6 update together
# --------------------------------------------------------------------------
print("\nSimulation running. View cameras at http://localhost:{}"
      "\nClose window or Ctrl+C to exit.".format(args.port))

capture_cbs = {i: make_capture_cb(i) for i, _ in viewports}

def schedule_batch():
    _staging_count[0] = 0
    _batch_ready[0] = False
    for i, vp in viewports:
        try:
            capture_viewport_to_buffer(vp.viewport_api, capture_cbs[i])
        except Exception:
            pass

schedule_batch()

try:
    while simulation_app.is_running():
        simulation_app.update()
        if _batch_ready[0]:
            schedule_batch()

except (KeyboardInterrupt, SystemExit):
    print("\nShutting down...")
except Exception as e:
    print(f"\n[error] Main loop: {e}")
finally:
    simulation_app.close()
