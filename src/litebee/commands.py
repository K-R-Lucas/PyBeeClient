from dataclasses import dataclass
from math import atan2, cos, pi, radians, sin

from . import types


@dataclass(slots=True)
class Command:
    flag: types.UnsignedLeb128
    attributes: dict[types.UnsignedLeb128, types.Attribute | None] | None

    def to_bytes(self):
        if self.attributes is None:
            data = b""
        else:
            data = (
                b"".join(
                    types.UnsignedLeb128(flag).to_bytes() + attr.to_bytes()
                    for flag, attr in sorted(
                        self.attributes.items(), key=lambda pair: pair[0]
                    )
                    if attr
                )
                or b""
            )

        return self.flag.to_bytes() + types.UnsignedLeb128(len(data)).to_bytes() + data


@dataclass(init=False, slots=True)
class Colour(Command):
    def __init__(self, colour: tuple[int, int, int]):
        r, g, b = colour

        super().__init__(
            flag=types.UnsignedLeb128(858),
            attributes={
                0x20: types.UnsignedLeb128(r),
                0x28: types.UnsignedLeb128(g),
                0x30: types.UnsignedLeb128(b),
            },
        )


@dataclass(init=False, slots=True)
class ColourGradient(Command):
    def __init__(self, colour: tuple[int, int, int], flicker: bool = False, flicker_freq: int = 2):
        r, g, b = colour

        super().__init__(
            flag=types.UnsignedLeb128(882),
            attributes={
                0x20: types.UnsignedLeb128(r),
                0x28: types.UnsignedLeb128(g),
                0x30: types.UnsignedLeb128(b),
                0x38: types.UnsignedLeb128(int(flicker)),
                0x40: types.UnsignedLeb128(flicker_freq),
            },
        )


@dataclass(init=False, slots=True)
class Calibrate(Command):
    """
    Calibrate the drone for <t> seconds. This must be the first command the drone receives.
    """

    def __init__(self):
        super().__init__(flag=types.UnsignedLeb128(810), attributes=None)


@dataclass(init=False, slots=True)
class Takeoff(Command):
    """
    Launch the drone to <height> cm over <t> secnods.
    """

    def __init__(self, height):
        super().__init__(
            flag=types.UnsignedLeb128(818),
            attributes={
                0x20: types.UnsignedLeb128(height),
            },
        )

    @property
    def height(self) -> int:
        return self.attributes.get(0x20, types.UnsignedLeb128()).value

    @height.setter
    def height(self, value: int):
        self.attributes[0x20] = types.UnsignedLeb128(value)

    def calculate_delta(self, t: float):
        return types.Vector3(0, 0, t * self.height)


@dataclass(init=False, slots=True)
class Land(Command):
    """
    Land the drone. <t> should not be changed from 3 seconds, though it seems to still work.
    """

    def __init__(self):
        super().__init__(flag=types.UnsignedLeb128(826), attributes=None)

    def calculate_delta(self, t: float, start_pos: types.Vector3):
        z = -t * start_pos.z
        return types.Vector3(0, 0, z)


@dataclass(init=False, slots=True)
class Move3D(Command):
    """
    Move the drone to position <pos(x, y, z)> cm over <t> seconds.
    """

    def __init__(self, target: types.Vector3):
        super().__init__(
            flag=types.UnsignedLeb128(834),
            attributes={
                0x20: types.UnsignedLeb128(target.x),
                0x28: types.UnsignedLeb128(target.y),
                0x30: types.UnsignedLeb128(target.z),
            },
        )

    @property
    def target(self) -> types.Vector3:
        return types.Vector3(
            x=self.attributes.get(0x20, types.UnsignedLeb128()).value,
            y=self.attributes.get(0x28, types.UnsignedLeb128()).value,
            z=self.attributes.get(0x30, types.UnsignedLeb128()).value,
        )

    @target.setter
    def target(self, value: types.Vector3):
        self.attributes[0x20] = types.UnsignedLeb128(value.x)
        self.attributes[0x28] = types.UnsignedLeb128(value.y)
        self.attributes[0x30] = types.UnsignedLeb128(value.z)

    def calculate_delta(self, t: float, start_pos):
        return t * (self.target - start_pos)


@dataclass(init=False, slots=True)
class Around(Command):
    """
    Move the drone around the specified <pos> by 180 degrees <half_num> times.
    """

    def __init__(
        self,
        origin: types.Vector2,
        clockwise: bool,
        half_rotations: int,
    ):

        super().__init__(
            flag=types.UnsignedLeb128(842),
            attributes={
                0x20: types.UnsignedLeb128(origin.x),
                0x28: types.UnsignedLeb128(origin.y),
                0x30: types.UnsignedLeb128(origin.z),
                0x38: types.UnsignedLeb128(int(clockwise)),
                0x40: types.UnsignedLeb128(half_rotations),
            },
        )

    @property
    def origin(self) -> types.Vector3:
        return types.Vector3(
            x=self.attributes.get(0x20, types.UnsignedLeb128()).value,
            y=self.attributes.get(0x28, types.UnsignedLeb128()).value,
            z=self.attributes.get(0x30, types.UnsignedLeb128()).value,
        )

    @origin.setter
    def origin(self, value: types.Vector3):
        self.attributes[0x20] = types.UnsignedLeb128(value.x)
        self.attributes[0x28] = types.UnsignedLeb128(value.y)
        self.attributes[0x30] = types.UnsignedLeb128(value.z)

    @property
    def direction(self) -> int:
        return -1 if self.attributes.get(0x38, types.UnsignedLeb128()) else 1

    @property
    def radians(self) -> float:
        return pi * self.attributes.get(0x40, types.UnsignedLeb128()).value

    def calculate_delta(self, t: float, start_pos: types.Vector3):
            delta = start_pos.xy - self.origin
            r = delta.length()
            ra = atan2(delta.y, delta.x)
            a = ra + self.direction * self.radians * t

            x = self.origin.x + r * cos(a)
            y = self.origin.y + r * sin(a)

            return types.Vector3(x, y, start_pos.z)


class AroundH(Command):
    """
    Move the drone around the specified <pos> in a spiral.
    """

    def __init__(self, origin: types.Vector2, height: float, clockwise: bool):
        super().__init__(
            flag=types.UnsignedLeb128(850),
            attributes={
                0x20: types.UnsignedLeb128(origin.x),
                0x28: types.UnsignedLeb128(origin.y),
                0x30: types.UnsignedLeb128(height),
                0x38: types.UnsignedLeb128(int(clockwise)),
            },
        )

    @property
    def direction(self) -> bool:
        return -1 if self.attributes.get(0x38, types.UnsignedLeb128()) else 1

    @property
    def origin(self) -> types.Vector2:
        return types.Vector3(
            x=self.attributes.get(0x20, types.UnsignedLeb128()).value,
            y=self.attributes.get(0x28, types.UnsignedLeb128()).value,
        )

    @origin.setter
    def origin(self, value: types.Vector2):
        self.attributes[0x20] = types.UnsignedLeb128(value.x)
        self.attributes[0x28] = types.UnsignedLeb128(value.y)

    @property
    def height(self) -> int:
        return self.attributes.get(0x30, types.UnsignedLeb128()).value

    @height.setter
    def height(self, value: float):
        self.attributes[0x30] = types.UnsignedLeb128(value)

    def calculate_delta(self, t: float, start_pos: types.Vector3):
            delta = start_pos.xy - self.origin
            r = delta.length()
            ra = atan2(delta.y, delta.x)
            a = ra + self.direction * pi * t

            x = self.origin.x + r * cos(a)
            y = self.origin.y + r * sin(a)
            z = start_pos.z + t * (self.height - start_pos.z)

            return types.Vector3(x, y, z)


class AroundD(Command):
    """
    Note that instead of a <height> parameter, the <pos> has an x, y, z (height)
    """

    def __init__(
        self,
        origin: types.Vector2,
        height: int,
        clockwise: bool,
        angle: int,
    ):
        super().__init__(
            flag=types.UnsignedLeb128(866),
            attributes={
                0x20: types.UnsignedLeb128(origin.x),
                0x28: types.UnsignedLeb128(origin.y),
                0x30: types.UnsignedLeb128(height),
                0x38: types.UnsignedLeb128(int(clockwise)),
                0x40: types.UnsignedLeb128(angle),
            },
        )

    @property
    def angle(self) -> float:
        return radians(self.attributes.get(0x40, types.UnsignedLeb128()).value)

    @property
    def direction(self) -> bool:
        return -1 if self.attributes.get(0x38, types.UnsignedLeb128()) else 1

    @property
    def origin(self) -> types.Vector2:
        return types.Vector2(
            x=self.attributes.get(0x20, types.UnsignedLeb128()).value,
            y=self.attributes.get(0x28, types.UnsignedLeb128()).value,
        )


    @origin.setter
    def origin(self, value: types.Vector2):
        self.attributes[0x20] = types.UnsignedLeb128(value.x)
        self.attributes[0x28] = types.UnsignedLeb128(value.y)

    @property
    def height(self) -> int:
        return self.attributes.get(0x30, types.UnsignedLeb128()).value

    @height.setter
    def height(self, value: int):
        self.attributes[0x30] = types.UnsignedLeb128(value)

    def calculate_delta(self, t: float, start_pos: types.Vector3):
        delta = start_pos.xy - self.origin
        r = delta.length()
        ra = atan2(delta.y, delta.x)
        a = ra + self.direction * self.angle * t

        x = self.origin.x + r * cos(a)
        y = self.origin.y + r * sin(a)
        z = start_pos.z + t * (self.height - start_pos.z)

        return types.Vector3(x, y, z)


class Curve3(Command):
    """
    Move the drone along a Bezier3 curve.
    """

    def __init__(self, target: types.Vector3, control: types.Vector3):
        super().__init__(
            flag=types.UnsignedLeb128(874),
            attributes={
                0x20: types.UnsignedLeb128(target.x),
                0x28: types.UnsignedLeb128(target.y),
                0x30: types.UnsignedLeb128(target.z),
                0x40: types.UnsignedLeb128(control.x),
                0x48: types.UnsignedLeb128(control.y),
                0x50: types.UnsignedLeb128(control.z),
            },
        )

    @property
    def target(self) -> types.Vector3:
        return types.Vector3(
            x=self.attributes.get(0x20, types.UnsignedLeb128()).value,
            y=self.attributes.get(0x28, types.UnsignedLeb128()).value,
            z=self.attributes.get(0x30, types.UnsignedLeb128()).value,
        )

    @target.setter
    def target(self, value: types.Vector3):
        self.attributes[0x20] = types.UnsignedLeb128(value.x)
        self.attributes[0x28] = types.UnsignedLeb128(value.y)
        self.attributes[0x30] = types.UnsignedLeb128(value.z)

    @property
    def control(self) -> types.Vector3:
        return types.Vector3(
            x=self.attributes.get(0x40, types.UnsignedLeb128()).value,
            y=self.attributes.get(0x48, types.UnsignedLeb128()).value,
            z=self.attributes.get(0x50, types.UnsignedLeb128()).value,
        )

    @control.setter
    def control(self, value: types.Vector3):
        self.attributes[0x40] = types.UnsignedLeb128(value.x)
        self.attributes[0x48] = types.UnsignedLeb128(value.y)
        self.attributes[0x50] = types.UnsignedLeb128(value.z)

    def calculate_delta(self, t: float, start_pos: types.Vector3):
        p0 = types.Vector3(0, 0, 0)
        p1 = self.control - start_pos
        p2 = self.target - start_pos
        return (1 - t) ** 2 * p0 + 2 * t * (1 - t) * p1 + t**2 * p2

    def update(self, start_pos: types.Vector3):
        a = start_pos + self.calculate_delta(1 / 3, start_pos)
        b = start_pos + self.calculate_delta(2 / 3, start_pos)

        self.attributes.update(
            {
                0x58: types.UnsignedLeb128(a.x),
                0x60: types.UnsignedLeb128(a.y),
                0x68: types.UnsignedLeb128(a.z),
                0x70: types.UnsignedLeb128(b.x),
                0x78: types.UnsignedLeb128(b.y),
                0x80: types.UnsignedLeb128(b.z),
            }
        )
