from litebee.utils import uleb128
from litebee.core import Command
from pygame.math import Vector3, Vector2
from math import pi, radians

class Calibrate(Command):
    """
    Calibrate the drone for <t> seconds. This must be the first command the drone receives.
    """
    __slots__ = [
        't'
    ]

    def __init__(self, t: float = 5.0):
        self.t = t

        params = [
            {
                "flag": 810,
                "value": 0,
                "type": "int"
            },
            {
                "flag": 0x08,
                "value": 10*t,
                "type": "int"
            }
        ]

        super().__init__(params)


class Takeoff(Command):
    """
    Launch the drone to <height> cm over <t> secnods.
    """
    __slots__ = [
        't',
        'h'
    ]

    def __init__(self, height: int = 100, t: float = 5.0):
        self.t = t
        self.h = height

        params = [
            {
                "flag": 818,
                "value": 1 + len(
                    uleb128.from_int(height)
                ),
                "type": "int"
            },
            {
                "flag": 0x20,
                "value": height,
                "type": "int"
            },
            {
                "flag": 0x08,
                "value": 10*t,
                "type": "int"
            },
            {
                "flag": 0x10,
                "value": 0x01,
                "type": "int"
            }
        ]

        super().__init__(params)


class Move3D(Command):
    """
    Move the drone to position <pos(x, y, z)> cm over <t> seconds.
    """
    __slots__ = [
        'target'
    ]

    def __init__(self, pos: Vector3, t: float = 10.0):
        if not isinstance(pos, Vector3):
            pos = Vector3(pos)
        
        self.target = pos

        p = Command([
            {
                "flag": 0x20,
                "value": pos.x,
                "type": "int"
            },
            {
                "flag": 0x28,
                "value": pos.y,
                "type": "int"
            },
            {
                "flag": 0x30,
                "value": pos.z,
                "type": "int"
            }
        ])
        
        params = [
            {
                "flag": 834,
                "value": p,
                "type": "command"
            },
            {
                "flag": 0x08,
                "value": 10*t,
                "type": "int"
            },
            {
                "flag": 0x10,
                "value": 0x0A,
                "type": "int"
            }
        ]

        super().__init__(params)


class Around(Command):
    """
    Move the drone around the specified <pos> by 180 degrees <half_num> times.
    """
    __slots__ = [
        't',
        "origin",
        "radians",
        "direction"
    ]

    def __init__(self, pos: Vector2, t: float = 10.0, half_num: int = 1, is_clockwise: bool = True):
        if not isinstance(pos, Vector3):
            pos = Vector3(pos)
        
        self.origin = pos
        self.radians = pi * half_num
        self.direction = -1 if is_clockwise else 1

        p = Command([
            {
                "flag": 0x20,
                "value": pos.x,
                "type": "int"
            },
            {
                "flag": 0x28,
                "value": pos.y,
                "type": "int"
            },
            {
                "flag": 0x30,
                "value": pos.z,
                "type": "int"
            },
            {
                "flag": 0x38,
                "value": int(is_clockwise),
                "type": "int"
            },
            {
                "flag": 0x40,
                "value": half_num,
                "type": "int"
            }
        ])

        params = [
            {
                "flag": 842,
                "value": p,
                "type": "command"
            },
            {
                "flag": 0x08,
                "value": 10*t,
                "type": "int"
            },
            {
                "flag": 0x10,
                "value": 0x0C,
                "type": "int"
            }
        ]

        super().__init__(params)


class AroundH(Command):
    """
    Move the drone around the specified <pos> in a spiral.
    """
    __slots__ = [
        't', 'h',
        "origin",
        "direction"
    ]

    def __init__(self, pos: Vector2, height: int = 100, t: float = 10.0, is_clockwise: bool = True):
        self.t = t
        self.h = height
        self.origin = Vector2(pos)
        self.direction = -1 if is_clockwise else 1

        p = Command([
            {
                "flag": 0x20,
                "value": self.origin.x,
                "type": "int"
            },
            {
                "flag": 0x28,
                "value": self.origin.y,
                "type": "int"
            },
            {
                "flag": 0x30,
                "value": height,
                "type": "int"
            },
            {
                "flag": 0x38,
                "value": int(is_clockwise),
                "type": "int"
            },
        ])

        params = [
            {
                "flag": 850,
                "value": p,
                "type": "command"
            },
            {
                "flag": 0x08,
                "value": 10*t,
                "type": "int"
            },
            {
                "flag": 0x10,
                "value": 0x0D,
                "type": "int"
            }
        ]

        super().__init__(params)


class AroundD(Command):
    """
    Note that instead of a <height> parameter, the <pos> has an x, y, z (height)
    """
    __slots__ = [
        't', 'h', 'a',
        "origin",
        "direction"
    ]

    def __init__(self, pos: Vector2, height: int = 100, angle: int = 100, t: float = 10.0, is_clockwise: bool = True):
        self.t = t
        self.h = height
        self.a = radians(angle)
        self.origin = Vector2(pos)
        self.direction = -1 if is_clockwise else 1

        p = Command([
            {
                "flag": 0x20,
                "value": self.origin.x,
                "type": "int"
            },
            {
                "flag": 0x28,
                "value": self.origin.y,
                "type": "int"
            },
            {
                "flag": 0x30,
                "value": height,
                "type": "int"
            },
            {
                "flag": 0x38,
                "value": int(is_clockwise),
                "type": "int"
            },
            {
                "flag": 0x40,
                "value": angle,
                "type": "int"
            }
        ])

        params = [
            {
                "flag": 866,
                "value": p,
                "type": "command"
            },
            {
                "flag": 0x08,
                "value": 10*t,
                "type": "int"
            },
            {
                "flag": 0x10,
                "value": 0x0E,
                "type": "int"
            }
        ]

        super().__init__(params)


class Land(Command):
    """
    Land the drone. <t> should not be changed from 3 seconds, though it seems to still work.
    """
    __slots__ = [
        't'
    ]

    def __init__(self, t: float = 3.0):
        self.t = t

        params = [
            {
                "flag": 826,
                "value": 0,
                "type": "int"
            },
            {
                "flag": 0x08,
                "value": 10*t,
                "type": "int"
            },
            {
                "flag": 0x10,
                "value": 0x02,
                "type": "int"
            }
        ]

        super().__init__(params)

class Curve3(Command):
    """
    Move the drone along a Bezier3 curve.
    """
    __slots__ = [
        't'
        "control",
        "target"
    ]

    def __init__(self, target_pos: Vector3, control_point_1: Vector3, t: float = 10.0):
        self.t = t
        self.control = Vector3(control_point_1)
        self.target = Vector3(target_pos)

        curve = Command([
            {
                "flag": 0x20,
                "value": self.target.x,
                "type": "int"
            },
            {
                "flag": 0x28,
                "value": self.target.y,
                "type": "int"
            },
            {
                "flag": 0x30,
                "value": self.target.z,
                "type": "int"
            },
            {
                "flag": 0x40,
                "value": self.control.x,
                "type": "int"
            },
            {
                "flag": 0x48,
                "value": self.control.y,
                "type": "int"
            },
            {
                "flag": 0x50,
                "value": self.control.z,
                "type": "int"
            }
        ])

        params = [
            {
                "flag": 874,
                "value": curve,
                "type": "command"
            },
            {
                "flag": 0x08,
                "value": 10*t,
                "type": "int"
            },
            {
                "flag": 0x10,
                "value": 0x0F,
                "type": "int"
            }
        ]

        super().__init__(params)