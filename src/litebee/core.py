from struct import pack
from uuid import uuid4
from os import path

from litebee.utils import uleb128


class Command:
    """
    The base class for all commands.
    """
    __slots__ = [
        "params",
        "bytes_"
    ]

    def __init__(self, params: list[dict]):
        self.params = params
        self.bytes_ = None

    def add_parameter(self, flag: int | None, value: int | bytes):
        self.params.append({
            "flag": flag,
            "value": value
        })
    
    def add_rgb(self, colour: tuple[int, int, int], t: float = 0.0):
        self.params.append({
            "flag": 0x1A,
            "value": RGB(colour, t),
            "type": "command"
        })

        self.params.append({
            "flag": 0x22,
            "value": RGB(colour, t),
            "type": "command"
        })

        return self

    def add_gradient(self, colour: tuple[int, int, int], t: float = 0.0, flicker: int = 0):
        self.params.append({
            "flag": 0x22,
            "value": RGBGradient(colour, t, flicker),
            "type": "command"
        })

        return self

    def get_bytes(self, force_recompile: bool = False):
        if (not force_recompile) and (self.bytes_ is not None):
            return self.bytes_
        
        params = b''
        for param in self.params:
            flag = param["flag"]
            value = param["value"]
            data_type = param["type"]
            flag_bytes = uleb128.from_int(flag)\
                         if flag is not None else b''

            match data_type:
                case "string":
                    value_bytes = uleb128.from_int(len(value))\
                                  + bytes(value, encoding="utf-8")
                
                case "command":
                    value = value.get_bytes()
                    value_bytes = uleb128.from_int(len(value)) + value
                
                case "int":
                    value_bytes = uleb128.from_int(value)

                case "float":
                    value_bytes = pack("<f", value)

                case _:
                    print(data_type, "is not handled...")
            
            params += flag_bytes + value_bytes

        self.bytes_ = params
        return params
    

class RGB(Command):
    """
    The backend Command object for RGB control. This should not be used directly.
    Use Command.add_rgb() to contorl lighting.
    """
    def __init__(self, colour: tuple[int, int, int], t: float = 0.0):
        c = Command([
            {
                "flag": 0x20,
                "value": colour[0],
                "type": "int"
            },
            {
                "flag": 0x28,
                "value": colour[1],
                "type": "int"
            },
            {
                "flag": 0x30,
                "value": colour[2],
                "type": "int"
            }
        ])

        params = [
            {
                "flag": 858,
                "value": c,
                "type": "command"
            },
            {
                "flag": 0x08,
                "value": 10*t,
                "type": "int"
            }
        ]

        super().__init__(params)


class RGBGradient(Command):
    def __init__(self, colour: tuple[int, int, int], t: float = 0.0, flicker: int = 0):
        c = Command([
            {
                "flag": 0x20,
                "value": colour[0],
                "type": "int"
            },
            {
                "flag": 0x28,
                "value": colour[1],
                "type": "int"
            },
            {
                "flag": 0x30,
                "value": colour[2],
                "type": "int"
            },
            {
                "flag": 0x38,
                "value": int(flicker > 0),
                "type": "int"
            },
            {
                "flag": 0x40,
                "value": flicker,
                "type": "int"
            }
        ])

        params = [
            {
                "flag": 882,
                "value": c,
                "type": "command"
            },
            {
                "flag": 0x08,
                "value": 10*t,
                "type": "int"
            },
            {
                "flag": 0x10,
                "value": 0x10,
                "type": "int"
            }
        ]

        super().__init__(params)


class  Drone(Command):
    """
    Initialise drone <number> at position <pos>
    """
    __slots__ = [
        "start_pos"
    ]

    def __init__(self, number: int, pos: tuple[float, float]):
        self.start_pos = pos
        params = [
            {
                "flag": 0x10,
                "value": number,
                "type": "int"
            },
            {
                "flag": 0x1D,
                "value": 0.01*pos[0],
                "type": "float"
            },
            {
                "flag": 0x2D,
                "value": 0.01*pos[1],
                "type": "float"
            },
            
        ]

        super().__init__(params)
    
    def add_command(self, command: Command):
        """
        Add a single command to the drone. All drones should have at least 
        the Calibrate, Takeoff and Land commands before being used in a show.
        """
        self.params.append({
            "flag": 0x32,
            "value": command,
            "type": "command"
        })

        return self
    
    def add_commands(self, *commands: list[Command]):
        """
        Add multiple commands to the drone. All drones should have at least 
        the Calibrate, Takeoff and Land commands before being used in a show.
        """
        for command in commands:
            self.add_command(command)
        
        return self


class Case(Command):
    """
    Initialize a new light show.
    By default, the UUID is randomly generated and the version number is set to 1.3.11
    """
    __slots__ = [
        "drone_count",
        "uuid",
        "name",
        "gx", "gy",
        "version",
        "start_pos"
    ]

    def __init__(self, name: str,\
                 gx: int, gy: int, version: str = "1.3.11", uuid: str = None):
        self.uuid = uuid or str(uuid4())
        self.name = name
        self.gx = gx
        self.gy = gy
        self.version = version

        params = [
            {
                "flag": 0x0A,
                "value": self.uuid,
                "type": "string"
            },
            {
                "flag": 0x12,
                "value": self.name,
                "type": "string"
            },
            {
                "flag": 0x18,
                "value": self.gx,
                "type": "int"
            },
            {
                "flag": 0x20,
                "value": self.gy,
                "type": "int"
            },
            {
                "flag": 0x2A,
                "value": self.version,
                "type": "string"
            }
        ]

        self.drone_count = 0
        super().__init__(params)

    def add_drone(self, start_pos: tuple[float, float] = None):
        """
        Add a drone to the light show.
        Takes a start position in metres.
        """
        if start_pos is None:
            x = 50 + 50*(self.drone_count%(2*self.gx - 1))
            y = 50 + 50*(self.drone_count//(2*self.gx - 1))

        else:
            x, y = start_pos

        self.drone_count += 1
        drone = Drone(self.drone_count, (x, y))

        self.params.append(
            {
                "flag": 0x32,
                "value": drone,
                "type": "command"
            }
        )

        return drone

    def save(self, file_path: str | None = None):
        if file_path is None:
            file_path = self.name
            
        if not file_path.endswith(".bin"):
            file_path += ".bin"

        with open(file_path, "wb") as file:
            file.write(self.get_bytes())