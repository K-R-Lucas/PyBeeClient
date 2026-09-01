import os
import pickle
from datetime import datetime
from math import pi, sqrt
from sys import setrecursionlimit

import numpy as np
from PIL import Image

from .types import Rect, Vector2, Vector3

setrecursionlimit(1000)


class ImageScanner:
    def __init__(self, image_path: str):
        if not image_path.lower().endswith(".png"):
            raise ValueError("Only PNG files can be scanned.")

        try:
            self.img = Image.open(image_path, formats=["PNG"])
        except TypeError:
            raise ValueError("Only PNG files can be scanned.")

        self.w, self.h = self.img.size
        self.points = None

    def __scan_pixels(
        self,
        pos: tuple[int, int],
        master: bool = True,
        results: dict | None = None,
        alpha_threshold: int = 10,
    ):
        if master:
            results: dict = {}

        elif results is None:
            raise ValueError("<results> can only be None if <master> is True.")

        colour = self.img.getpixel(pos)

        if not isinstance(colour, tuple):
            raise TypeError()

        r, g, b, a = colour

        if a >= alpha_threshold:
            results[pos] = (r, g, b)
            x, y = pos

            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:
                    if (dx == 0) and (dy == 0):
                        continue

                    new_pos = (x + dx, y + dy)
                    if (new_pos in results) or (
                        (x < 0) or (x >= self.w) or (y < 0) or (y >= self.h)
                    ):
                        continue

                    self.__scan_pixels(new_pos, False, results, alpha_threshold)

        return results

    def mul_colour(self, colour: tuple[int, int, int], factor: float):
        return tuple(int(i * factor) for i in colour)

    def get_points(self, alpha_threshold: int = 10) -> dict[tuple, tuple]:
        """
        Generate a dictionary of points from the provided image.

        params:
            alpha_threshold: int = 10 -> Any alpha value greater than this will be included in the scan.
        """
        averages = []
        self.points = {}

        min_x = float("inf")
        max_x = -float("inf")
        min_y = float("inf")
        max_y = -float("inf")
        min_r = float("inf")
        max_r = -float("inf")
        all_results = {}

        for yi in range(self.h):
            for xi in range(self.w):
                results = self.__scan_pixels((xi, yi), alpha_threshold=alpha_threshold)

                if not results:
                    continue

                if (xi, yi) in all_results:
                    continue

                all_results.update(results)
                keys = results.keys()
                n = len(keys)

                X = 0
                Y = 0

                for x, y in keys:
                    X += x
                    Y += y

                X /= n
                Y /= n

                R = 0
                G = 0
                B = 0

                for r, g, b in results.values():
                    R += r
                    G += g
                    B += b

                R /= n
                G /= n
                B /= n

                r = 2 * sqrt(n / pi)
                averages.append(((X, Y), (R, G, B), r))
                min_r = min(min_r, r)
                max_r = max(max_r, r)
                min_x = min(min_x, X)
                max_x = max(max_x, X)
                min_y = min(min_y, Y)
                max_y = max(max_y, Y)

        for pos, colour, r in averages:
            self.points[
                Vector3(
                    x=1 - (pos[0] - min_x) / (max_x - min_x),
                    y=1 - (r - min_r) / (max_r - min_r),
                    z=(pos[1] - min_y) / (max_y - min_y),
                )
            ] = colour

        return (max_x - min_x) / (max_y - min_y), self.points

    def save_points(self, output_fp: str):
        assert self.points is not None

        with open(output_fp, "wb") as file:
            pickle.dump(self.points, file)

    def load_points(self, input_fp: str):
        assert os.path.exists(input_fp)

        with open(input_fp, "rb") as file:
            self.points = pickle.load(file)

        return self.points


def convert_time(stamp: datetime) -> int:
    return int((stamp.timestamp() + 62135636400) * 1e7)


def scale_points(
    points: dict[Vector3, tuple[int, int, int]],
    bounding_box: Rect,
    depth_range: tuple[float, float],
) -> dict[Vector3, tuple[int, int, int]]:
    new: dict[Vector3, tuple[int, int, int]] = {}
    min_depth, max_depth = depth_range

    for pos, colour in points.items():
        new_pos: Vector2 = bounding_box.bottom_left + pos.xz * bounding_box.size
        new[
            Vector3(new_pos.x, pos.y * (max_depth - min_depth) + min_depth, new_pos.y)
        ] = colour

    return new


def rotate_points(
    points: list[Vector3], anchor_point: Vector3, axis: Vector3, angle: float
):
    """<axis> must be normal."""
    return [
        anchor_point + (point - anchor_point).rotate_around(axis, angle)
        for point in points
    ]


def explode_points(points: list[Vector3], anchor_point: Vector3, factor: float):
    deltas = [point - anchor_point for point in points]
    return [anchor_point + delta * factor for delta in deltas]


def find_centre(points: list[Vector3]) -> Vector3:
    total = Vector3(0, 0, 0)
    for point in points:
        total += point

    return total / len(points)


def find_normal(points: list[Vector3]) -> Vector3:
    point_arr = np.array([[point.x, point.y, point.z] for point in points])
    centroid = np.mean(point_arr, axis=0)
    centred = point_arr - centroid
    _, _, vt = np.linalg.svd(centred, full_matrices=False)
    return Vector3([float(i) for i in vt[-1, :]])


def pathfind_drones(drones: list, points: list[Vector3]):
    point_map = []

    for point in points:
        distances = []

        for drone in drones:
            distances.append(point.distance_squared_to(drone.sim_pos))

        if distances:
            point_map.append((min(distances), point))

    drones = drones.copy()
    for _, point in sorted(point_map, key=lambda x: x[0]):
        closest = None
        min_d = float("inf")

        for drone in drones:
            d = drone.sim_pos.distance_squared_to(point)

            if d < min_d:
                min_d = d
                closest = drone.id

        for i in range(len(drones) - 1, -1, -1):
            if drones[i].id == closest:
                drone = drones.pop(i)
                break

        yield drone, point
