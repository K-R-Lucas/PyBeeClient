from sys import setrecursionlimit
import pygame as pg
import pickle
import os

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
        self.img = pg.image.load(image_path).convert_alpha()
        self.w, self.h = self.img.get_size()
        self.points = None

    def __scan_pixels(self, x: int, y: int, master: bool = True, results: dict = None):
        if master:
            results = dict()
        
        pos = (x, y)
        if pos in results:
            return None

        if (x < 0) or (x >= self.w) or (y < 0) or (y >= self.h):
            return None

        colour = self.img.get_at((x, y))
        if colour.a:
            results[pos] = (colour.r, colour.g, colour.b)

            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:
                    if (dx == 0) and (dy == 0):
                        continue

                    self.__scan_pixels(x+dx, y+dy, False, results)
        
        return results
    

    def get_points(self) -> dict[tuple, tuple]:
        averages = dict()

        for yi in range(self.h):
            for xi in range(self.w):
                results = self.__scan_pixels(xi, yi)

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
        
        self.points = averages
        return averages
    
    def save_points(self, output_fp: str):
        assert self.points is not None

        with open(output_fp, "wb") as file:
            pickle.dump(self.points, file)
        
    def load_points(self, input_fp: str):
        assert os.path.exists(input_fp)

        with open(input_fp, "rb") as file:
            self.points = pickle.load(file)
        
        return self.points