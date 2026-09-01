import json
from os import path

from .types import String, UnsignedLeb128


class GameFrameworkSetting:
    __slots__ = ["fp", "header", "settings"]

    def __init__(self, config_fp: str | None = None):
        if config_fp is None:
            self.fp = path.expanduser(
                "~/AppData/LocalLow/创客火/LiteBeeClien/GameFrameworkSetting.dat"
            )

        else:
            self.fp = config_fp

    @staticmethod
    def process_value(value: UnsignedLeb128 | String) -> str | int | dict | list:
        if isinstance(value, UnsignedLeb128):
            return value.value

        try:
            return json.loads(value.value)
        except json.decoder.JSONDecodeError:
            return value.value

    def read_settings(self) -> dict:
        output = []

        with open(self.fp, "rb") as file:
            self.header = file.read(5)

            while char := file.read(1):
                size = ord(char)

                if size > 127:
                    size = UnsignedLeb128.from_bytes(char + file.read(1))

                output.append(String(file.read(size).decode("utf-8")))

        self.settings = {
            output[i].value: self.process_value(output[i + 1])
            for i in range(0, len(output), 2)
        }

        return self.settings

    def write_settings(self):
        output = b""
        for key, value in self.settings.items():
            v = str(value).encode("utf-8")

            output += (
                UnsignedLeb128(len(key)).to_bytes()
                + key.encode("utf-8")
                + UnsignedLeb128(len(v)).to_bytes()
                + v
            )

        with open(self.fp, "wb") as file:
            file.write(self.header + output)

    def __getitem__(self, key: str):
        return self.settings[key]

    def __setitem__(self, key: str, value):
        self.settings[key] = value
