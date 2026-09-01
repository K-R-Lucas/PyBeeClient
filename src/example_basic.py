from random import choice, random

from litebee import Case
from litebee.instructions import *

show = Case("Basic Light Show Example", 5, 5)
colours = [
    (255, 0, 0),
    (0, 255, 0),
    (0, 0, 255),
    (255, 255, 0),
    (255, 0, 255),
    (0, 255, 255),
]

for i in range(3):
    x = 25 + i * 225
    y = 25
    z = 100

    show.add_drone((x, y)).add_instructions(
        Calibrate().add_rgb(choice(colours)),
        Takeoff(z),
        Move3D((x, y + random() * 400, z)).add_rgb(choice(colours)),
        Land(),
    )

show.save()  # If no name is provided, it will use the case name. i.e: "Basic Light Show Example.bin"
