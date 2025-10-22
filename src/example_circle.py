from math import sin, cos, pi
from pygame import Vector3

from litebee.commands import *
from litebee import core

drone_show = core.Case(
    "Python Light Show - Circle", 5, 5
)

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
    
    start_a = -0.5*(i-2)*pi
    radius = 200
    centre = Vector3(250, 250, 300)
    drone = drone_show.add_drone(p).add_commands(
        Calibrate(),
        Takeoff(300, 1),
        Move3D(
            centre + radius*Vector3(cos(start_a), sin(start_a), 0), 1.0
        )
    )

    for i in range(18):
        a = start_a - i * pi/20
        drone.add_command(
            Move3D(
                centre + radius*Vector3(cos(a), sin(a), 0), 0.5
            )
        )

    drone.add_command(Land())

drone_show.save()

