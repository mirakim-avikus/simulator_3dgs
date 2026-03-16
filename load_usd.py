"""Load a USD file and set up a fisheye camera using Isaac Sim."""

import argparse
import math
import os
import sys

import numpy as np
from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Load a USD file in Isaac Sim")
parser.add_argument("usd_path", help="Path to the .usd/.usda/.usdz file (local path or Nucleus URL)")
parser.add_argument("--headless", action="store_true", help="Run without GUI")
args = parser.parse_args()

# Resolve to absolute path for local files
usd_path = args.usd_path
if not usd_path.startswith("omniverse://") and not os.path.isabs(usd_path):
    usd_path = os.path.abspath(usd_path)

simulation_app = SimulationApp({"headless": args.headless})

import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdLux

# --------------------------------------------------------------------------
# OCamCalib fisheye parameters
# --------------------------------------------------------------------------
# cam2world polynomial (ss): z(rho) = ss[0] + ss[1]*rho + ss[2]*rho^2 + ...
OCAM_SS = [-2.907366e+02, 0.0, 6.915871e-04, 5.741715e-08, 3.065713e-10]
# world2cam polynomial (invpol): rho(theta) = invpol[0] + invpol[1]*theta + ...
OCAM_INVPOL = [590.937502, 522.678139, 114.990712, 23.814065,
               45.831575, 49.789384, 20.772449, 3.232227]
# Center (row, col) in OCamCalib convention
OCAM_CENTER_ROW, OCAM_CENTER_COL = 361.303803, 639.669324
# Affine parameters (c, d, e)
OCAM_AFFINE_C, OCAM_AFFINE_D, OCAM_AFFINE_E = 1.001320, -0.001614, 0.001861
# Image size
OCAM_HEIGHT, OCAM_WIDTH = 720, 1280


def ocam_to_ftheta_coeffs(ss, center_row, center_col, height, width, num_samples=500):
    """Convert OCamCalib cam2world polynomial to f-theta polynomial coefficients.

    OCamCalib cam2world: z(rho) = ss[0] + ss[1]*rho + ss[2]*rho^2 + ...
    F-theta model:       theta(r) = k0 + k1*r + k2*r^2 + k3*r^3 + k4*r^4

    Returns (ftheta_coeffs, max_fov_deg).
    """
    # Maximum pixel radius from center to image corner
    max_r = math.sqrt(max(center_col, width - center_col) ** 2 +
                      max(center_row, height - center_row) ** 2)

    # Sample rho values from 0 to max_r
    rho = np.linspace(0, max_r, num_samples)

    # Compute z(rho) using OCamCalib cam2world polynomial
    z = np.zeros_like(rho)
    for i, coeff in enumerate(ss):
        z += coeff * (rho ** i)

    # Angle from optical axis: theta = atan2(rho, -z)
    # (z is negative for forward-looking rays in OCamCalib convention)
    theta = np.arctan2(rho, -z)

    # Fit degree-4 polynomial: theta(rho) = k0 + k1*rho + ... + k4*rho^4
    ftheta_coeffs = np.polyfit(rho, theta, deg=4)[::-1].tolist()  # reverse to [k0, k1, k2, k3, k4]

    max_fov_deg = float(np.degrees(theta[-1])) * 2  # full FOV angle
    return ftheta_coeffs, max_fov_deg


def create_fisheye_camera(stage, prim_path="/World/FisheyeCamera",
                          position=(0, 0, 1.5), look_at=(0, 0, 0)):
    """Create a fisheye camera prim with OCamCalib parameters using f-theta distortion model."""
    # Convert OCamCalib to f-theta
    ftheta_coeffs, max_fov = ocam_to_ftheta_coeffs(
        OCAM_SS, OCAM_CENTER_ROW, OCAM_CENTER_COL, OCAM_HEIGHT, OCAM_WIDTH
    )

    # Create camera prim
    camera = UsdGeom.Camera.Define(stage, Sdf.Path(prim_path))

    # Set transform
    xform = UsdGeom.Xformable(camera.GetPrim())
    xform.ClearXformOpOrder()
    translate_op = xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(*position))

    # Point camera toward look_at target
    forward = Gf.Vec3d(*look_at) - Gf.Vec3d(*position)
    forward_n = forward.GetNormalized()
    # Compute rotation matrix (camera looks down -Z in USD)
    cam_z = -forward_n
    world_up = Gf.Vec3d(0, 0, 1)
    cam_x = (world_up ^ cam_z).GetNormalized()  # cross product
    if cam_x.GetLength() < 1e-6:
        world_up = Gf.Vec3d(0, 1, 0)
        cam_x = (world_up ^ cam_z).GetNormalized()
    cam_y = (cam_z ^ cam_x).GetNormalized()
    rot_mat = Gf.Matrix4d(1.0)
    rot_mat.SetRow3(0, cam_x)
    rot_mat.SetRow3(1, cam_y)
    rot_mat.SetRow3(2, cam_z)
    xform_mat = rot_mat.SetTranslateOnly(Gf.Vec3d(0, 0, 0))
    rotation = xform_mat.ExtractRotation()
    quat = rotation.GetQuat()
    orient_op = xform.AddOrientOp()
    orient_op.Set(Gf.Quatf(quat.GetReal(), *quat.GetImaginary()))

    # Basic camera properties
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10000.0))
    # Set a nominal focal length (will be overridden by distortion model)
    camera.GetFocalLengthAttr().Set(2.0)
    camera.GetHorizontalApertureAttr().Set(4.0)
    camera.GetVerticalApertureAttr().Set(4.0 * OCAM_HEIGHT / OCAM_WIDTH)

    # Apply f-theta lens distortion model
    prim = camera.GetPrim()
    prim.ApplyAPI("OmniLensDistortionFthetaAPI")
    prim.GetAttribute("omni:lensdistortion:model").Set("ftheta")

    # Set f-theta parameters
    # Optical center: OCamCalib (row, col) -> Isaac Sim (x=col, y=row)
    prim.GetAttribute("omni:lensdistortion:ftheta:opticalCenter").Set(
        Gf.Vec2f(OCAM_CENTER_COL, OCAM_CENTER_ROW)
    )
    prim.GetAttribute("omni:lensdistortion:ftheta:nominalWidth").Set(float(OCAM_WIDTH))
    prim.GetAttribute("omni:lensdistortion:ftheta:nominalHeight").Set(float(OCAM_HEIGHT))
    prim.GetAttribute("omni:lensdistortion:ftheta:maxFov").Set(max_fov)

    # Set polynomial coefficients
    for i, coeff in enumerate(ftheta_coeffs):
        prim.GetAttribute(f"omni:lensdistortion:ftheta:k{i}").Set(float(coeff))

    # Store original OCamCalib parameters as custom attributes for reference
    ns = "ocamcalib"
    prim.CreateAttribute(f"{ns}:ss", Sdf.ValueTypeNames.FloatArray, False).Set(OCAM_SS)
    prim.CreateAttribute(f"{ns}:invpol", Sdf.ValueTypeNames.FloatArray, False).Set(OCAM_INVPOL)
    prim.CreateAttribute(f"{ns}:center", Sdf.ValueTypeNames.Float2, False).Set(
        Gf.Vec2f(OCAM_CENTER_ROW, OCAM_CENTER_COL)
    )
    prim.CreateAttribute(f"{ns}:affine", Sdf.ValueTypeNames.Float3, False).Set(
        Gf.Vec3f(OCAM_AFFINE_C, OCAM_AFFINE_D, OCAM_AFFINE_E)
    )
    prim.CreateAttribute(f"{ns}:imageSize", Sdf.ValueTypeNames.Int2, False).Set(
        Gf.Vec2i(OCAM_HEIGHT, OCAM_WIDTH)
    )

    print(f"Created fisheye camera at {prim_path}")
    print(f"  Image: {OCAM_WIDTH}x{OCAM_HEIGHT}, center=({OCAM_CENTER_COL:.1f}, {OCAM_CENTER_ROW:.1f})")
    print(f"  Max FOV: {max_fov:.1f}°")
    print(f"  F-theta coeffs: {[f'{c:.6e}' for c in ftheta_coeffs]}")

    return camera


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
# Open the USD stage
usd_context = omni.usd.get_context()
result = usd_context.open_stage(usd_path)
if not result:
    print(f"Failed to open USD file: {usd_path}")
    simulation_app.close()
    sys.exit(1)

print(f"Loaded USD: {usd_path}")

stage = usd_context.get_stage()

# Print stage hierarchy
print("\n--- Stage Hierarchy ---")
for prim in stage.Traverse():
    print(f"  {prim.GetPath()} ({prim.GetTypeName()})")

# Add a light if none exists
lights = [p for p in stage.Traverse() if p.IsA(UsdLux.DistantLight) or p.IsA(UsdLux.DomeLight)]
if not lights:
    light = UsdLux.DistantLight.Define(stage, Sdf.Path("/DefaultLight"))
    light.CreateIntensityAttr(500)
    print("Added default distant light")

# Create fisheye camera
fisheye_cam = create_fisheye_camera(stage)

# Run simulation loop
print("\nSimulation running. Close the window to exit.")
while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
