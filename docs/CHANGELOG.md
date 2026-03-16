# Changelog

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
