from dataclasses import dataclass, field

from . import commands, types


@dataclass(slots=True)
class Instruction:
    flag: types.UnsignedLeb128
    attributes: dict[int, types.Attribute] = field(default_factory=dict)
    commands: list[commands.Command] = field(default_factory=list)
    instructions: "list[Instruction]" = field(default_factory=list)

    start_pos: types.Vector3 | None = field(init=False, default_factory=types.Vector3)
    end_pos: types.Vector3 | None = field(init=False, default_factory=types.Vector3)

    @property
    def duration(self) -> float:
        return self.attributes.get(0x08, types.UnsignedLeb128()).value / 10.0

    @duration.setter
    def duration(self, value: float):
        self.attributes[0x08] = types.UnsignedLeb128(int(value * 10))

    def add_rgb(self, colour: tuple[int, int, int], delay: float = 0):
        self.instructions.extend(
            [Colour(0x1A, colour, delay), Colour(0x22, colour, delay)]
        )
        return self

    def calculate_pos(self, t: float) -> types.Vector3: ...

    def to_bytes(self):
        data = (
            b"".join(
                command.to_bytes()
                for command in sorted(self.commands, key=lambda cmd: cmd.flag.value)
            )
            + b"".join(
                types.UnsignedLeb128(flag).to_bytes() + attr.to_bytes()
                for flag, attr in sorted(
                    self.attributes.items(), key=lambda pair: pair[0]
                )
                if attr
            )
            + b"".join(
                instruction.to_bytes()
                for instruction in sorted(
                    self.instructions, key=lambda inst: inst.flag.value
                )
            )
        )

        return self.flag.to_bytes() + types.UnsignedLeb128(len(data)).to_bytes() + data


@dataclass(init=False, slots=True)
class Drone(Instruction):
    start_pos: types.Vector3
    sim_time: float
    sim_pos: types.Vector3
    sim_ref_pos: types.Vector3
    sim_instruction: int
    sim_instruction_start_time: float

    def __init__(self, number: int, pos: types.Vector3):
        self.attributes = {
            0x10: types.UnsignedLeb128(number + 1),
            0x1D: types.Float(0.01 * pos.x),
            0x2D: types.Float(0.01 * pos.y),
        }

        self.flag = types.UnsignedLeb128(0x32)
        self.instructions = []
        self.commands = []
        self.start_pos = pos.copy()
        self.duration = 0
        self.reset_simulation()

    def calculate_key_points(self):
        if len(self.instructions) == 0:
            return

        self.reset_simulation()

        for instruction in self.instructions:
            instruction.start_pos = self.sim_ref_pos
            instruction.end_pos = instruction.calculate_pos(1)
            self.sim_ref_pos = instruction.end_pos.copy()

    def add_instruction(self, instruction: "Instruction"):
        self.instructions.append(instruction)
        self.calculate_key_points()
        self.sim_pos = self.sim_ref_pos.copy()
        return self

    def add_instructions(self, *instructions: "Instruction"):
        for instruction in instructions:
            self.add_instruction(instruction)
        return self

    def reset_simulation(self):
        self.sim_time = 0
        self.sim_pos = self.start_pos.copy()
        self.sim_ref_pos = self.start_pos.copy()
        self.sim_instruction = 0
        self.sim_instruction_start_time = 0

        for instruction in self.instructions:
            instruction.start_pos = None
            instruction.end_pos = None

    def simulate_step(self, dt: float):
        if not self.instructions:
            return False

        if self.sim_instruction == len(self.instructions):
            return False

        instruction = self.instructions[self.sim_instruction]
        if instruction.start_pos is None:
            instruction.start_pos = self.sim_ref_pos.copy()

        instruction_elapsed = self.sim_time - self.sim_instruction_start_time

        if instruction_elapsed >= instruction.duration:
            instruction.end_pos = instruction.calculate_pos(1)
            self.sim_ref_pos = instruction.end_pos.copy()
            self.sim_instruction_start_time += instruction.duration
            self.sim_instruction += 1
            self.sim_time += dt
            return self.simulate_step(0)

        progress = instruction_elapsed / instruction.duration
        self.sim_pos = instruction.calculate_pos(progress)
        self.sim_time += dt
        return True

    @property
    def id(self):
        return self.attributes.get(0x10, types.UnsignedLeb128()).value - 1


@dataclass(init=False, slots=True)
class Colour(Instruction):
    def __init__(self, flag: int, colour: tuple[int, int, int], delay: float = 0):
        super().__init__(
            flag=types.UnsignedLeb128(flag),
            attributes={},
            commands=[commands.Colour(colour)],
        )

        self.duration = delay

    def calculate_pos(self, _):
        return self.start_pos.copy()


@dataclass(init=False, slots=True)
class Calibrate(Instruction):
    def __init__(self, duration: float = 5.0):
        super().__init__(
            flag=types.UnsignedLeb128(0x32), commands=[commands.Calibrate()]
        )
        self.duration = duration

    def calculate_pos(self, _):
        return self.start_pos.copy()


@dataclass(init=False, slots=True)
class Takeoff(Instruction):
    path: commands.Takeoff

    def __init__(self, height: int = 100, duration: float = 5.0):
        self.path = commands.Takeoff(height)
        super().__init__(
            flag=types.UnsignedLeb128(0x32),
            attributes={0x10: types.UnsignedLeb128(0x01)},
            commands=[self.path],
        )
        self.duration = duration

    def calculate_pos(self, t: float):
        return self.start_pos + self.path.calculate_delta(t)


@dataclass(init=False, slots=True)
class Land(Instruction):
    path: commands.Land

    def __init__(self, duration: float = 3.0):
        self.path = commands.Land()
        super().__init__(
            flag=types.UnsignedLeb128(0x32),
            attributes={0x10: types.UnsignedLeb128(0x02)},
            commands=[self.path],
        )
        self.duration = duration

    def calculate_pos(self, t: float):
        return self.start_pos + self.path.calculate_delta(t, self.start_pos)


@dataclass(init=False, slots=True)
class Move3D(Instruction):
    path: commands.Move3D

    def __init__(self, target: types.Vector3, duration: float = 10.0):
        self.path = commands.Move3D(target)
        super().__init__(
            flag=types.UnsignedLeb128(0x32),
            attributes={0x10: types.UnsignedLeb128(0x0A)},
            commands=[self.path],
        )
        self.duration = duration

    def calculate_pos(self, t: float):
        return self.start_pos + self.path.calculate_delta(t, self.start_pos)


@dataclass(init=False, slots=True)
class Around(Instruction):
    path: commands.Around

    def __init__(
        self,
        origin: types.Vector2,
        clockwise: bool = True,
        half_rotations: int = 1,
        duration: float = 10.0,
    ):
        self.path = commands.Around(origin, clockwise, half_rotations)
        super().__init__(
            flag=types.UnsignedLeb128(0x32),
            attributes={0x10: types.UnsignedLeb128(0x0C)},
            commands=[self.path],
        )
        self.duration = duration

    def calculate_pos(self, t: float):
        return self.start_pos + self.path.calculate_delta(t)


@dataclass(init=False, slots=True)
class AroundH(Instruction):
    path: commands.AroundH

    def __init__(
        self, origin: types.Vector2, clockwise: bool = True, duration: float = 10.0
    ):
        self.path = commands.AroundH(origin, clockwise)
        super().__init__(
            flag=types.UnsignedLeb128(0x32),
            attributes={0x10: types.UnsignedLeb128(0x0D)},
            commands=[self.path],
        )
        self.duration = duration

    def calculate_pos(self, t: float):
        return self.start_pos + self.path.calculate_delta(t, self.start_pos)


@dataclass(init=False, slots=True)
class AroundD(Instruction):
    path: commands.AroundD

    def __init__(
        self,
        origin: types.Vector2,
        clockwise: bool = True,
        degrees: int = 100,
        duration: float = 10.0,
    ):
        self.path = commands.AroundD(origin, clockwise, degrees)
        super().__init__(
            flag=types.UnsignedLeb128(0x32),
            attributes={0x10: types.UnsignedLeb128(0x0E)},
            commands=[self.path],
        )
        self.duration = duration

    def calculate_pos(self, t: float):
        return self.start_pos + self.path.calculate_delta(t, self.start_pos)


@dataclass(init=False, slots=True)
class Curve3(Instruction):
    path: commands.Curve3

    def __init__(
        self, target: types.Vector3, control: types.Vector3, duration: float = 10.0
    ):
        self.path = commands.Curve3(target, control)
        super().__init__(
            flag=types.UnsignedLeb128(0x32),
            attributes={0x10: types.UnsignedLeb128(0x0F)},
            commands=[self.path],
        )
        self.duration = duration

    def calculate_pos(self, t: float):
        return self.start_pos + self.path.calculate_delta(t, self.start_pos)
