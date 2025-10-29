from litebee.commands import *
from litebee.core import Case
from litebee.utils import ImageScanner
from time import perf_counter
import pickle
import os

scanner = ImageScanner("dist/image-removebg-preview.png")
print(len(scanner.get_points()))

# drone_show = Case(
#     "Python Light Show", 20, 15, 100
# )


# drones = [drone_show.add_drone() for _ in range(80)]

# for drone, (pos, colour) in zip(drones, points.items()):
#     drone.add_commands(
#         Calibrate(),
#         Takeoff(50, 1),
#         Move3D((pos[0], 500, 1000 - pos[1]+50)).add_rgb(colour, 10)
#     )

# for pos, colour in points.items():
#     drone_show.add_drone().add_commands(
#         Move3D((pos[0], 1500, 1000 - pos[1]+50)).add_rgb(colour, 10),
#         Land()
#     )

# # Leaving the save() function empty will use the Case.name instead.
# drone_show.save("dist/show.bin")
