import pygame as pg
from sys import setrecursionlimit

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
        pg.display.set_mode((1, 1,), pg.HIDDEN)
        self.img = pg.image.load(image_path).convert()
        self.w, self.h = self.img.get_size()

    def __scan_pixels(self, x: int, y: int, v_threshold: int = 30, master: bool = True, results: dict = None):
        if master:
            results = dict()

        if (x < 0) or (x >= self.w) or (y < 0) or (y >= self.h):
            return None
        
        pos = (x, y)
        if pos in results:
            return None

        colour = self.img.get_at((x, y))
        if colour.hsva[2] > v_threshold:
            results[pos] = (colour.r, colour.g, colour.b)

            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:
                    self.__scan_pixels(x+dx, y+dy, v_threshold, False, results)
        
        return results
    

    def get_points(self, v_threshold: int = 30) -> dict[tuple, tuple]:
        averages = dict()

        for yi in range(self.h):
            for xi in range(self.w):
                results = self.__scan_pixels(xi, yi, v_threshold)

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

                averages[(X, Y)] = (R, G, B)
        
        return averages