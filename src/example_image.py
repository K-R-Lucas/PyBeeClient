from litebee.commands import *
from litebee.core import Case
from litebee.utils import ImageScanner


image = ImageScanner("test.png")
points = image.get_points()

show = Case("Python Light Show - Image", 10, 10)

for point, colour in points.items():
    drone = show.add_drone()
    start = drone.start_pos

    drone.add_commands(
        Calibrate(),
        Takeoff(),
        Move3D((point[0], 100, 1000 - point[1])),
        Move3D((start[0], start[1], 1000 - point[1])),
        Land()
    )

show.save()
