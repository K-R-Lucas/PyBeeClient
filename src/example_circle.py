from math import cos, pi, sin

from litebee import Case
from litebee.instructions import *
from litebee.types import Vector3

drone_show = Case("Python Light Show - Circle", 5, 5)

for i in range(4):
    match i:
        case 0:
            p = (75, 75)

        case 1:
            p = (75, 500 - 75)

        case 2:
            p = (500 - 75, 500 - 75)

        case 3:
            p = (500 - 75, 75)

    start_a = -0.5 * (i - 2) * pi
    radius = 200
    centre = Vector3(250, 250, 300)
    drone = drone_show.add_drone(p).add_instructions(
        Calibrate(),
        Takeoff(300, 1),
        Move3D(centre + radius * Vector3(cos(start_a), sin(start_a), 0), 1.0),
    )

    for i in range(18):
        a = start_a - i * pi / 20
        drone.add_instruction(Move3D(centre + radius * Vector3(cos(a), sin(a), 0), 0.5))

    drone.add_instruction(Land())

drone_show.save()
