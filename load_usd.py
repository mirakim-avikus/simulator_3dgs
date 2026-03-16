"""Load and display a USD file using Isaac Sim."""

import argparse
import os
import sys

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
from pxr import Sdf, UsdLux

# Open the USD stage
usd_context = omni.usd.get_context()
result = usd_context.open_stage(usd_path)
if not result:
    print(f"Failed to open USD file: {usd_path}")
    simulation_app.close()
    sys.exit(1)

print(f"Loaded USD: {usd_path}")

# Print stage hierarchy
stage = usd_context.get_stage()
print("\n--- Stage Hierarchy ---")
for prim in stage.Traverse():
    print(f"  {prim.GetPath()} ({prim.GetTypeName()})")

# Add a light if none exists
lights = [p for p in stage.Traverse() if p.IsA(UsdLux.DistantLight) or p.IsA(UsdLux.DomeLight)]
if not lights:
    light = UsdLux.DistantLight.Define(stage, Sdf.Path("/DefaultLight"))
    light.CreateIntensityAttr(500)
    print("Added default distant light")

# Run simulation loop
print("\nSimulation running. Close the window to exit.")
while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
