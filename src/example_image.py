from litebee import Case
from litebee.instructions import *
from litebee.types import Rect, Vector3
from litebee.utils import ImageScanner, scale_points

# Use a dot-art image with a transparent background (PNG)
image = ImageScanner("dot_art.png")
points = image.get_points(alpha_threshold=254)
scaled_points = scale_points(points, Rect(0.5, 0.5, 9, 9), (9.5, 0.5))

show = Case("Python Light Show - Image", 10, 10)

# Create a drone for every point on the image and assign it to the coordinates.
for point, colour in scaled_points.items():
    drone = show.add_drone()
    start = drone.start_pos

    drone.add_instructions(
        Calibrate(),
        Takeoff(),
        Move3D(point),
        # You can add another Move3D(point, duration=...) here
        # to make the drones stay in the same position for some time.
        Move3D(start + Vector3(0, 0, 100)).add_rgb(colour),
        Land(),
    )

show.save()
