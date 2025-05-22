# PyBeeClient
##### A Python module designed to take Python code and compile it into LiteBeeClient-compatible bytecode.
This project is not designed to be a full replacement to LiteBeeClient, nor is it intended to have implementations for every single feature of LiteBeeClient. The following components are currently working for LBC v1.3.9 - v1.3.11:
- Create a case.
- Add multiple drones to one case.
- Commands:
    - Calibrate
    - Takeoff
    - Move3D
    - Land
    - +RGB on any of these.

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

drone_show.save("Python Light Show")


```
