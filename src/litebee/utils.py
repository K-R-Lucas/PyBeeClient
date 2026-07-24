import os
import pickle
from datetime import datetime
from sys import setrecursionlimit

from PIL import Image

setrecursionlimit(1000)


class uleb128:
    @staticmethod
    def from_int(input_int: float):
        assert input_int >= 0

        byte_list = []
        value = int(input_int)

        while True:
            byte = 0b01111111 & value
            value >>= 7

            if value != 0:
                byte |= 0b10000000

            byte_list.append(byte)

            if value == 0:
                break

        return bytes(byte_list)

    @staticmethod
    def to_int(input_bytes: bytes):
        result = 0
        shift = 0

        for byte in input_bytes:
            result |= (0b01111111 & byte) << shift

            if not (0b10000000 & byte):
                break

            shift += 7

        return result


class ImageScanner:
    def __init__(self, image_path: str):
        if not image_path.lower().endswith(".png"):
            raise ValueError("Only PNG files can be scanned.")

        try:
            self.img = Image.open(image_path, formats=["image/png"])
        except Exception:
            raise ValueError("Only PNG files can be scanned.")

        self.w, self.h = self.img.size
        self.points = None

    def __scan_pixels(
        self,
        x: int,
        y: int,
        master: bool = True,
        results: dict | None = None,
        alpha_threshold: int = 10,
    ):
        if master:
            results = dict()

        elif results is None:
            raise ValueError("<results> can only be None if <master> is True.")

        pos = (x, y)
        if pos in results:
            return None

        if (x < 0) or (x >= self.w) or (y < 0) or (y >= self.h):
            return None

        colour = self.img.getpixel((x, y))

        if not isinstance(colour, tuple):
            raise ValueError()

        r, g, b, a = colour

        if a > alpha_threshold:
            results[pos] = (r, g, b)

            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:
                    if (dx == 0) and (dy == 0):
                        continue

                    self.__scan_pixels(x + dx, y + dy, False, results, alpha_threshold)

        return results

    def mul_colour(self, colour: tuple[int, int, int], factor: float):
        return tuple(int(i * factor) for i in colour)

    def get_points(
        self,
        alpha_threshold: int = 10,
        auto_brightness: bool = True,
        max_depth: int = 1000,
    ) -> dict[tuple, tuple]:
        """
        Generate a dictionary of points from the provided image. If <auto_brightness> is True,
        the brightness of drones will be proportional to the number of pixels in a dot.
        <auto_brightness_exp> is the exponent of the proportionality. Lower values will result in smaller differences in brightness.
        """
        averages = list()
        self.points = dict()

        min_x = float("inf")
        max_x = -float("inf")
        min_y = float("inf")
        max_y = -float("inf")
        max_n = -float("inf")

        for yi in range(self.h):
            for xi in range(self.w):
                results = self.__scan_pixels(xi, yi, alpha_threshold=alpha_threshold)

                if not results:
                    continue

                X = 0
                Y = 0
                keys = results.keys()
                n = len(keys)

                for x, y in keys:
                    X += x
                    Y += y

                X /= n
                Y /= n

                R = 0
                G = 0
                B = 0

                colours = results.values()

                for r, g, b in colours:
                    R += r
                    G += g
                    B += b

                R /= n
                G /= n
                B /= n

                averages.append(((X, Y), (R, G, B), n))

                if n > max_n:
                    max_n = n

                if X < min_x:
                    min_x = X

                elif X > max_x:
                    max_x = X

                if Y < min_y:
                    min_y = Y

                elif Y > max_y:
                    max_y = Y

        for pos, colour, n in averages:
            self.points[
                (pos[0] - min_x) / (max_x - min_x),
                (pos[1] - min_y) / (max_y - min_y),
                (1 - (n / max_n)),
            ] = colour

        return self.points

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
