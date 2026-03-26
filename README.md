# PyBeeClient

### A Python library designed to replace the need to use LiteBee Client's built-in editor.

This project is not designed to be a full replacement to LiteBeeClient, nor is it intended to have implementations for every single feature of LiteBeeClient. The following components are currently tested on version 1.3.9:
All commands (except for Curve4) and basic usage have been implemented, this includes:

- Calibrate
- Takeoff
- Land
- Move3D
- Around
  - Around(H)
  - Around(D)
- RGB
- RGBGradient
- Curve3

Examples can be found in the `src/` directory.

### Advanced Features
- Light shows can be simulated within Python, LiteBee is not required at all. I have also implemented an experimental collision avoidance alogirthm (`litebee.core.Case.fix_collisions()`). This will take a completed show, simulate it in discrete steps, then attempt to resolve collisions.
- Dot-art images can be used to generate frames in a light show. The image requires a transparent background. The code will detect clumps of non-transparent pixels, average the colours, then return all the positions in a dictionary where keys are positions and values are the averaged colours.
- Though not implemented in the library, I have dabbled and had success with reading vertices from an OBJ file and loading them in as drone positions.
- LiteBee only validates fields on user input, not when values are loaded directly from a file, meaning you can go beyond certain limits. I've tested the following:
    - Excessive grid sizes.
    - Large drone counts (3600+).
    - Drones without commands (either no commands at all or not implementing takeoffs, land or calibrate).
        - You can also actually start a show with no takeoff/movement commands. The drones will not start their motors. This is good for testing lights.

 - Under `litebee.config` there are tools to read/write LiteBee Client's GameFrameworkSetting file which, most notably, contains the DroneRecord list. This list maps the drones' DroneIDs to indexes and is used to order drones in the control module.
