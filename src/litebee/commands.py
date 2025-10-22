from litebee.utils import uleb128
from litebee.core import Command

class Calibrate(Command):
    """
    Calibrate the drone for <t> seconds. This must be the first command the drone receives.
    """
    def __init__(self, t: float = 5.0):
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
    def __init__(self, height: int = 100, t: float = 5.0):
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
    def __init__(self, pos: tuple[int, int, int], t: float = 10.0):
        p = Command([
            {
                "flag": 0x20,
                "value": pos[0],
                "type": "int"
            },
            {
                "flag": 0x28,
                "value": pos[1],
                "type": "int"
            },
            {
                "flag": 0x30,
                "value": pos[2],
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
    def __init__(self, pos: tuple[int, int, int], t: float = 10.0, half_num: int = 1, is_clockwise: bool = True):
        p = Command([
            {
                "flag": 0x20,
                "value": pos[0],
                "type": "int"
            },
            {
                "flag": 0x28,
                "value": pos[1],
                "type": "int"
            },
            {
                "flag": 0x30,
                "value": pos[2],
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
    Note that instead of a <height> parameter, the <pos> has an x, y, z (height)
    """
    def __init__(self, pos: tuple[int, int, int], t: float = 10.0, is_clockwise: bool = True):
        p = Command([
            {
                "flag": 0x20,
                "value": pos[0],
                "type": "int"
            },
            {
                "flag": 0x28,
                "value": pos[1],
                "type": "int"
            },
            {
                "flag": 0x30,
                "value": pos[2],
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
    def __init__(self, pos: tuple[int, int, int], t: float = 10.0, degree: int = 0, is_clockwise: bool = True):
        p = Command([
            {
                "flag": 0x20,
                "value": pos[0],
                "type": "int"
            },
            {
                "flag": 0x28,
                "value": pos[1],
                "type": "int"
            },
            {
                "flag": 0x30,
                "value": pos[2],
                "type": "int"
            },
            {
                "flag": 0x38,
                "value": int(is_clockwise),
                "type": "int"
            },
            {
                "flag": 0x40,
                "value": degree,
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
    def __init__(self, t: float = 3.0):
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
