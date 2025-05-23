# PyBeeClient
##### A Python module designed to take Python code and compile it into LiteBeeClient-compatible bytecode.
This project is not designed to be a full replacement to LiteBeeClient, nor is it intended to have implementations for every single feature of LiteBeeClient. The following components are currently working for LBC v1.3.9 - v1.3.11:

All commands (except for Curve) and basic usage have been implemented, this includes:
- Calibrate
- Takeoff
- Land
- Move3D
- Around
    - Around(H)
    - Around(D)
- RGB
- RGBGradient


Basic usage:
```python
from litebee.commands import *
from litebee.core import Case

drone_show = Case(
    "Python Light Show", 5, 5
)

drone_show.add_commands(
    Calibrate(),
    Takeoff().add_rgb((255, 0, 0)),
    Move3D((250, 250, 250), 2.0),
    Land().add_rgb((0, 0, 255))
)

"""
Commands can also be added individually;
drone_show.add_command(Land())
"""

# Leaving the save() function empty will use the Case.name instead.
drone_show.save()


```
