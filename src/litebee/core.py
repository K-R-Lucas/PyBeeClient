from pygame.math import Vector3
from datetime import datetime
from struct import pack
from uuid import uuid4
from os import path

import json

from litebee.utils import uleb128, convert_time

class Command:
    """
    The base class for all commands.
    """
    __slots__ = [
        "params",
        "bytes_",
        't',
        "start_pos",
        "end_pos",
        "ref_pos"
    ]

    def __init__(self, params: list[dict], start_pos: Vector3 = None):
        self.params = params
        self.bytes_ = None
        self.start_pos = start_pos
        self.end_pos = None

        if not hasattr(self, 't'):
            self.t = 0
    
    def set_time(self, t: float):
        self.t = t

        for param in self.params:
            if param["flag"] == 0x08:
                param["value"] = 10*t
            
            elif param["type"] == "command":
                param["value"].set_time(t)
    
    def change_time(self, t: float):
        self.t += t

        for param in self.params:
            if param["flag"] == 0x08:
                param["value"] = 10*self.t
            
            elif param["type"] == "command":
                param["value"].change_time(t)

    def add_parameter(self, flag: int | None, value: int | bytes, type: str):
        self.params.append({
            "flag": flag,
            "value": value,
            "type": type
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
                    if hasattr(value, "update"):
                        value.update()
                    
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
    
    def calculate_delta(self, t: float) -> Vector3: ...
    def update(self) -> None: ...

    def __repr__(self):
        return str(
            {slot: getattr(self, slot) for slot in self.__slots__ if hasattr(self, slot)}
        )
    
    def __getitem__(self, flag: int) -> dict:
        for param in self.params:
            if param["flag"] == flag:
                return param
        

        raise KeyError(f"Flag 0x{hex(flag)[2:].upper()} not found")

    def __setitem__(self, flag: int, new_param: dict):
        for param in self.params:
            if param["flag"] == flag:
                param.update(new_param)
                return

        raise KeyError(f"Flag 0x{hex(flag)[2:].upper()} not found")

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


class Drone(Command):
    """
    Initialise drone <number> at position <pos>
    """
    __slots__ = [
        "commands",
        "start_pos",
        "simulated_time",
        "simulated_pos",
        "simulated_ref_pos",
        "simulated_command",
        "simulated_command_start_time",
        "id"
    ]

    def __init__(self, number: int, pos: Vector3):
        self.commands: list[Command] = list()
        self.start_pos = Vector3(pos)
        self.reset_simulation()
        self.id = number

        params = [
            {
                "flag": 0x10,
                "value": number,
                "type": "int"
            },
            {
                "flag": 0x1D,
                "value": 0.01*self.start_pos.x,
                "type": "float"
            },
            {
                "flag": 0x2D,
                "value": 0.01*self.start_pos.y,
                "type": "float"
            },
            
        ]

        super().__init__(params, start_pos=self.start_pos)

    def calculate_key_points(self):
        if len(self.commands) == 0:
            return

        self.reset_simulation()

        for command in self.commands:
            command.start_pos = self.simulated_ref_pos
            command.end_pos = command.start_pos + command.calculate_delta(1)
            self.simulated_ref_pos = command.end_pos
    
    def add_command(self, command: Command):
        """
        Add a single command to the drone. All drones should have at least 
        the Calibrate, Takeoff and Land commands before being used in a show.
        """
        
        self.commands.append(command)
        self.calculate_key_points()

        if hasattr(command, "update"):
            command.update()

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

    def calculate_duration(self):
        return sum(
            c.t for c in self.commands
        )
    
    def reset_simulation(self):
        self.simulated_time = 0
        self.simulated_pos = self.start_pos.copy()
        self.simulated_ref_pos = self.start_pos.copy()
        self.simulated_command = 0
        self.simulated_command_start_time = 0

        for command in self.commands:
            command.start_pos = None
            command.end_pos = None

    def simulate_step(self, dt: float):
        n = len(self.commands)

        if n == 0:
            return False
    
        if (self.commands[-1].end_pos is not None) and (self.simulated_command == n-1):
            return False
        
        cmd = self.commands[self.simulated_command]
        
        if cmd.start_pos is None:
            cmd.start_pos = self.simulated_ref_pos

        if self.simulated_time >= self.simulated_command_start_time + cmd.t:
            self.simulated_ref_pos += cmd.calculate_delta(1)
            cmd.end_pos = self.simulated_ref_pos

            if self.simulated_command + 1 < n:
                self.simulated_command += 1
                self.simulated_command_start_time += cmd.t
                return True
            else:
                return False
        
        cmd_time = self.simulated_time - self.simulated_command_start_time
        cmd_progress = cmd_time/cmd.t

        self.simulated_pos = self.simulated_ref_pos + cmd.calculate_delta(cmd_progress)
        self.simulated_time += dt
        return True


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
        "start_pos",
        "takeoff_spacing",
        "takeoff_w",
        "takeoff_h",
        "drones"
    ]

    def __init__(self, name: str,\
                 gx: int, gy: int, takeoff_spacing: int = 50, version: str = "1.3.11", uuid: str = None):
        self.uuid = uuid or str(uuid4())
        self.name = name
        self.gx = gx
        self.gy = gy
        self.version = version
        self.takeoff_spacing = takeoff_spacing
        self.takeoff_w = self.gx*100 / self.takeoff_spacing - 1
        self.takeoff_h = self.gy*100 / self.takeoff_spacing - 1
        self.drones: list[Drone] = list()

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
            x = 0.5*(100*self.gx - int(self.takeoff_w)*self.takeoff_spacing) + self.takeoff_spacing * (self.drone_count %  int(self.takeoff_w + 1))
            y = 0.5*(100*self.gy - int(self.takeoff_h)*self.takeoff_spacing) + self.takeoff_spacing * (self.drone_count // int(self.takeoff_w + 1))
        else:
            x, y = start_pos

        self.drone_count += 1
        drone = Drone(self.drone_count, Vector3(x, y, 0))
        self.drones.append(drone)

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
    
    def save_and_import(self, litebee_save_dir: str = None):
        if litebee_save_dir is None:
            litebee_save_dir = path.expanduser("~/AppData/LocalLow/创客火/LiteBeeClient/DesignCase/")
        
        config_dir = path.join(litebee_save_dir, "Config.txt")

        with open(config_dir, 'r') as file:
            config_data = json.load(file)
        
        if isinstance(config_data, list):
            for packet in config_data:
                packet.update({
                    "$type": "LittleBee.DesignCaseInfo, Assembly-CSharp"
                })

            config_data = {
                "$type": "System.Collections.Generic.List`1[[LittleBee.DesignCaseInfo, Assembly-CSharp]], mscorlib",
                "$values": config_data
            }

        t = convert_time(datetime.now())
        config_data["$values"].append({
            "$type": "LittleBee.DesignCaseInfo, Assembly-CSharp",
            "caseName": self.name,
            "caseID": self.uuid,
            "createTimeTicks": t,
            "lastTimeTicks": t,
            "curAudioID": None
        })

        with open(config_dir, 'w') as file:
            json.dump(config_data, file)

        self.save(
            path.join(litebee_save_dir, f"{self.uuid}.bin")
        )
    
    def reset_simulation_state(self):
        for drone in self.drones:
            drone.reset_simulation()
    
    def fix_collisions(self, resolution: float, correction_scale: float = 1.0, collision_radius: float = 60.0):
        from litebee.commands import Curve3, Move3D

        self.reset_simulation_state()
        drones = self.drones
        t = 0

        while drones:=[drone for drone in drones if drone.simulate_step(resolution)]:
            t += resolution
            n = len(drones)

            for i in range(n):
                d1 = drones[i]

                for j in range(i+1, n):
                    d2 = drones[j]

                    while d1.simulated_pos.distance_squared_to(d2.simulated_pos) < collision_radius**2:
                        if d1.simulated_ref_pos.distance_squared_to(d2.simulated_ref_pos) < (0.5*collision_radius)**2:
                            break

                        command = d1.commands[d1.simulated_command]

                        if isinstance(command, Curve3):
                            p1: Vector3 = command.control
                            p2: Vector3 = command.target

                            delta = p2 - p1.project(p2)
                            adj = correction_scale*resolution*delta

                            command[874]["value"][0x40] = {
                                "value": p1.x - adj.x,
                            }

                            command[874]["value"][0x48] = {
                                "value": p1.y - adj.y,
                            }

                            command[874]["value"][0x50] = {
                                "value": p1.z - adj.z,
                            }

                            command.control -= adj

                            t1 = 0
                            d1.reset_simulation()
                            while t1 < t:
                                d1.simulate_step(resolution)
                                t1 += resolution

                        elif isinstance(command, Move3D):
                            command.change_time(resolution)
                            d1.reset_simulation()

                            t1 = 0
                            while t1 < t:
                                d1.simulate_step(resolution)
                                t1 += resolution