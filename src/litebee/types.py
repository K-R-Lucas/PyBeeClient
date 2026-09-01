from collections.abc import Iterable
from dataclasses import dataclass
from math import cos, radians, sin, sqrt
from struct import pack
from typing import Any, overload


@dataclass(slots=True)
class UnsignedLeb128:
    value: int = 0

    def __bool__(self):
        return bool(self.value)

    def to_bytes(self):
        assert self.value >= 0

        byte_list = []
        value = int(self.value)

        while True:
            byte: int = 0b01111111 & value
            value >>= 7

            if value != 0:
                byte |= 0b10000000

            byte_list.append(byte)

            if value == 0:
                break

        return bytes(byte_list)

    @staticmethod
    def from_bytes(data: bytes) -> "UnsignedLeb128":
        result = 0
        shift = 0

        for byte in data:
            result |= (0b01111111 & byte) << shift

            if not (0b10000000 & byte):
                break

            shift += 7

        return UnsignedLeb128(result)


@dataclass(slots=True)
class String:
    value: str = ""

    def __bool__(self):
        return bool(self.value)

    def to_bytes(self):
        n = UnsignedLeb128(len(self.value))
        return n.to_bytes() + self.value.encode("utf-8")


@dataclass(slots=True)
class Float:
    value: float

    def __bool__(self):
        return bool(self.value)

    def to_bytes(self):
        return pack("<f", self.value)


@dataclass(slots=True)
class Attribute:
    flag: UnsignedLeb128
    value: Any

    def __bool__(self):
        return bool(self.value)

    def to_bytes(self):
        if self.value:
            return self.flag.to_bytes() + self.value.to_bytes()
        else:
            return b""


class Vector2:
    __slots__ = ("x", "y")
    x: float
    y: float

    @overload
    def __init__(self, x: float = 0, y: float = 0) -> None: ...

    @overload
    def __init__(self, x: Iterable[float]) -> None: ...

    def __init__(self, x: float | Iterable[float] = 0, y: float = 0) -> None:
        if isinstance(x, Iterable):
            self.x, self.y = x
        else:
            self.x, self.y = x, y

    def __mul__(self, other: "float | Vector2"):
        if isinstance(other, Vector2):
            return Vector2(x=self.x * other.x, y=self.y * other.y)

        return Vector2(x=self.x * other, y=self.y * other)

    def __truediv__(self, other: float):
        return Vector2(x=self.x / other, y=self.y / other)

    def __sub__(self, other: "Vector2"):
        return Vector2(x=self.x - other.x, y=self.y - other.y)

    def __add__(self, other: "Vector2"):
        return Vector2(x=self.x + other.x, y=self.y + other.y)

    def __imul__(self, other: float):
        self.x *= other
        self.y *= other
        return self

    def __itruediv__(self, other: float):
        self.x /= other
        self.y /= other
        return self

    def __iadd__(self, other: "Vector2"):
        self.x += other.x
        self.y += other.y
        return self

    def __isub__(self, other: "Vector2"):
        self.x -= other.x
        self.y -= other.y
        return self

    def copy(self) -> "Vector2":
        return Vector2(self.x, self.y)

    def __repr__(self) -> str:
        return f"Vector2(x={self.x}, y={self.y})"


class Vector3:
    __slots__ = ["x", "y", "z"]
    x: float
    y: float
    z: float

    @overload
    def __init__(self, x: float = 0, y: float = 0, z: float = 0) -> None: ...

    @overload
    def __init__(self, x: Iterable[float]) -> None: ...

    @overload
    def __init__(self, x: "Vector3") -> None: ...

    def __init__(
        self, x: "float | Iterable[float] | Vector3" = 0, y: float = 0, z: float = 0
    ) -> None:
        if isinstance(x, Vector3):
            self.x, self.y, self.z = x.x, x.y, x.z
        elif isinstance(x, Iterable):
            self.x, self.y, self.z = x
        else:
            self.x, self.y, self.z = x, y, z

    def __mul__(self, other: float):
        return Vector3(x=self.x * other, y=self.y * other, z=self.z * other)

    def __rmul__(self, other: float):
        return Vector3(x=self.x * other, y=self.y * other, z=self.z * other)

    def __truediv__(self, other: float):
        return Vector3(x=self.x / other, y=self.y / other, z=self.z / other)

    def __sub__(self, other: "Vector3"):
        return Vector3(x=self.x - other.x, y=self.y - other.y, z=self.z - other.z)

    def __rsub__(self, other: "Vector3"):
        return Vector3(x=other.x - self.x, y=other.y - self.y, z=other.z - self.z)

    def __add__(self, other: "Vector3"):
        return Vector3(x=self.x + other.x, y=self.y + other.y, z=self.z + other.z)

    def __radd__(self, other: "Vector3"):
        return Vector3(x=other.x + self.x, y=other.y + self.y, z=other.z + self.z)

    def __imul__(self, other: float):
        self.x *= other
        self.y *= other
        self.z *= other

    def __itruediv__(self, other: float):
        self.x /= other
        self.y /= other
        self.z /= other

    def __iadd__(self, other: "Vector3"):
        self.x += other.x
        self.y += other.y
        self.z += other.z
        return self

    def __isub__(self, other: "Vector3"):
        self.x -= other.x
        self.y -= other.y
        self.z -= other.z
        return self

    @property
    def xy(self):
        return Vector2(self.x, self.y)

    @property
    def xz(self):
        return Vector2(self.x, self.y)

    @property
    def yz(self):
        return Vector2(self.x, self.y)

    def copy(self) -> "Vector3":
        return Vector3(self.x, self.y, self.z)

    def distance_squared_to(self, other: "Vector3") -> float:
        return pow(self.x - other.x, 2) + pow(self.y - other.y, 2)

    def distance_to(self, other: "Vector3") -> float:
        return sqrt(self.distance_squared_to(other))

    def length_squared(self) -> float:
        return self.x * self.x + self.y * self.y

    def length(self) -> float:
        return sqrt(self.lenth())

    def dot(self, other: "Vector3") -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def project(self, other: "Vector3"):
        return other * (self.dot(other) / other.length_squared())

    def cross(self, other: "Vector3") -> "Vector3":
        return Vector3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )

    def normalise(self):
        return self / self.length()

    def rotate_around(self, axis: "Vector3", angle: float) -> "Vector3":
        a = radians(angle)
        return (
            self * cos(a)
            + axis.cross(self) * sin(a)
            + axis * axis.dot(self) * (1 - cos(a))
        )

    def __repr__(self) -> str:
        return f"Vector3(x={self.x}, y={self.y}, z={self.z})"


class Rect:
    __slots__ = ("height", "width", "x", "y")
    x: float
    y: float
    width: float
    height: float

    def __init__(self, x: float, y: float, width: float, height: float):
        """The bottom left corner is the reference point."""
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    @property
    def size(self):
        return Vector2(self.width, self.height)

    @size.setter
    def size(self, v: Vector2):
        self.width = v.x
        self.height = v.y

    @property
    def left(self):
        return self.x

    @left.setter
    def left(self, v: float):
        self.x = v

    @property
    def right(self):
        return self.x + self.width

    @right.setter
    def right(self, v: float):
        self.x = v - self.width

    @property
    def top(self):
        return self.y - self.height

    @top.setter
    def top(self, v: float):
        self.y = v + self.height

    @property
    def bottom(self):
        return self.y

    @bottom.setter
    def bottom(self, v: float):
        self.y = v

    @property
    def top_left(self):
        return Vector2(self.x, self.y)

    @top_left.setter
    def top_left(self, v: Vector2):
        self.x = v.x
        self.y = v.y - self.height

    @property
    def top_right(self):
        return Vector2(self.x + self.width, self.y)

    @top_right.setter
    def top_right(self, v: Vector2):
        self.x = v.x - self.width
        self.y = v.y - self.height

    @property
    def bottom_left(self):
        return Vector2(self.x, self.y)

    @bottom_left.setter
    def bottom_left(self, v: Vector2):
        self.x = v.x
        self.y = v.y

    @property
    def bottom_right(self):
        return Vector2(self.x + self.width, self.y)

    @bottom_right.setter
    def bottom_right(self, v: Vector2):
        self.x = v.x - self.width
        self.y = v.y

    @property
    def centre(self):
        return Vector2(self.x + 0.5 * self.width, self.y + 0.5 * self.height)

    @centre.setter
    def centre(self, v: Vector2):
        self.x = v.x - 0.5 * self.width
        self.y = v.y - 0.5 * self.height

    def __mul__(self, other: float) -> "Rect":
        return Rect(self.x, self.y, self.width * other, self.height * other)

    def __imul__(self, other: float) -> None:
        self.width *= other
        self.height *= other

    def __truediv__(self, other: float) -> "Rect":
        return Rect(self.x, self.y, self.width / other, self.height / other)

    def __itruediv__(self, other: float) -> None:
        self.width /= other
        self.height /= other

    def __sub__(self, other: "Rect") -> "Rect | None":
        left = max(self.left, other.left)
        right = min(self.right, other.right)
        top = max(self.top, other.top)
        bottom = min(self.bottom, other.bottom)

        if (left < right) and (top < bottom):
            return Rect(left, top, right - left, bottom - top)
