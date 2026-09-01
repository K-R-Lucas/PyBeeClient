import json
from dataclasses import dataclass
from datetime import datetime
from os import path
from uuid import uuid4

from . import types
from .commands import Command
from .instructions import Drone, Instruction
from .utils import convert_time


@dataclass(init=False, slots=True)
class Case(Instruction):
    """
    Initialize a new light show.
    By default, the UUID is randomly generated and the version number is set to 1.3.11
    params:
        name: str -> The title used for the Case in LiteBeeClient.
        gx: int -> The width of the grid in metres.
        gy: int -> The depth of the grid in metres.
        takeoff_spacing: int = 50 -> The gap between drones when automatically spaced.
        version: str = "1.3.11" -> The LiteBeeClient version number to include in the .bin file.
        uuid: str = None -> Used by LiteBeeClient to identify the Case.
    """

    drone_count: int
    drones: list[Drone]
    takeoff_h: float
    takeoff_w: float
    takeoff_spacing: int

    def __init__(
        self,
        name: str,
        gx: int,
        gy: int,
        takeoff_spacing: int = 50,
        version: str = "1.3.11",
        uuid: str | None = None,
    ):
        self.attributes = {
            0x0A: types.String(uuid or str(uuid4())),
            0x12: types.String(name),
            0x18: types.UnsignedLeb128(gx),
            0x20: types.UnsignedLeb128(gy),
            0x2A: types.String(version),
        }

        self.commands = [
            Command(
                flag=types.UnsignedLeb128(0x52),
                attributes={
                    0x10: types.UnsignedLeb128(self.gx * 100),
                    0x20: types.UnsignedLeb128(self.gy * 100),
                },
            ),
            Command(
                flag=types.UnsignedLeb128(0x3A),
                attributes={0x08: types.UnsignedLeb128(0x01)},
            ),
        ]

        self.instructions = []
        self.takeoff_spacing = takeoff_spacing
        self.takeoff_w = self.gx * 100 / self.takeoff_spacing - 1
        self.takeoff_h = self.gy * 100 / self.takeoff_spacing - 1
        self.drones: list[Drone] = []
        self.drone_count = 0

    @property
    def name(self) -> str:
        return self.attributes.get(0x12, types.String()).value

    @name.setter
    def name(self, value: str):
        self.attributes[0x12] = types.String(value)

    @property
    def version(self) -> str:
        return self.attributes.get(0x2A, types.String()).value

    @version.setter
    def version(self, value: str):
        self.attributes[0x2A] = types.String(value)

    @property
    def uuid(self) -> str:
        return self.attributes(0x0A, types.String()).value

    @uuid.setter
    def uuid(self, value: str):
        self.attributes[0x0A] = types.String(value)

    @property
    def gx(self) -> int:
        return self.attributes.get(0x18, types.UnsignedLeb128()).value

    @gx.setter
    def gx(self, value: int):
        self.attributes[0x18] = types.UnsignedLeb128(value)

    @property
    def gy(self) -> int:
        return self.attributes.get(0x20, types.UnsignedLeb128()).value

    @gy.setter
    def gy(self, value: int):
        self.attributes[0x20] = types.UnsignedLeb128(value)

    def to_bytes(self) -> bytes:
        return (
            b"".join(
                types.UnsignedLeb128(flag).to_bytes() + attr.to_bytes()
                for flag, attr in sorted(
                    self.attributes.items(), key=lambda pair: pair[0]
                )
            )
            + b"".join(
                instruction.to_bytes()
                for instruction in sorted(
                    self.instructions, key=lambda inst: inst.flag.value
                )
            )
            + b"".join(
                command.to_bytes()
                for command in sorted(self.commands, key=lambda cmd: cmd.flag.value)
            )
        )

    def add_drone(self, start_pos: tuple[float, float] | None = None):
        """
        Add a drone to the light show.
        Takes a start position in centimetres.
        """
        if start_pos is None:
            x = 0.5 * (
                100 * self.gx - int(self.takeoff_w) * self.takeoff_spacing
            ) + self.takeoff_spacing * (self.drone_count % int(self.takeoff_w + 1))
            y = 0.5 * (
                100 * self.gy - int(self.takeoff_h) * self.takeoff_spacing
            ) + self.takeoff_spacing * (self.drone_count // int(self.takeoff_w + 1))
        else:
            x, y = start_pos

        drone = Drone(self.drone_count, types.Vector3(x, y, 0))
        self.drones.append(drone)
        self.instructions.append(drone)
        self.drone_count += 1

        return drone

    def save(self, file_path: str | None = None):
        if file_path is None:
            file_path = self.name

        if not file_path.endswith(".bin"):
            file_path += ".bin"

        with open(file_path, "wb") as file:
            file.write(self.to_bytes())

    def save_and_import(self, litebee_save_dir: str | None = None):
        if litebee_save_dir is None:
            litebee_save_dir = path.expanduser(
                "~/AppData/LocalLow/创客火/LiteBeeClient/DesignCase/"
            )

        config_dir = path.join(litebee_save_dir, "Config.txt")

        with open(config_dir, "r") as file:
            config_data = json.load(file)

        if isinstance(config_data, list):
            for packet in config_data:
                packet.update({"$type": "LittleBee.DesignCaseInfo, Assembly-CSharp"})

            config_data = {
                "$type": "System.Collections.Generic.List`1[[LittleBee.DesignCaseInfo, Assembly-CSharp]], mscorlib",
                "$values": config_data,
            }

        t = convert_time(datetime.now())  # noqa: DTZ005
        config_data["$values"].append(
            {
                "$type": "LittleBee.DesignCaseInfo, Assembly-CSharp",
                "caseName": self.name,
                "caseID": self.uuid,
                "createTimeTicks": t,
                "lastTimeTicks": t,
                "curAudioID": None,
            }
        )

        with open(config_dir, "w") as file:
            json.dump(config_data, file)

        self.save(path.join(litebee_save_dir, f"{self.uuid}.bin"))

    def reset_simulation_state(self):
        for drone in self.drones:
            drone.reset_simulation()
