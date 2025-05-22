from litebee.commands import *
from litebee.core import Case
from random import choice, random

show = Case("Python Light Show - Basic", 5, 5)
colours = [
    (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255), (0, 255, 255)
]

for i in range(3):
    x = 25 + i*225
    y = 25
    z = 100

    show.add_drone((x, y)).add_commands(
        Calibrate().add_rgb(choice(colours)),
        Takeoff(z),
        Move3D((x, y + random() * 400, z)).add_rgb(choice(colours)),
        Land()
    )

show.save()
