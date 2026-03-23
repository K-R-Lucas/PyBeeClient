from litebee.utils import uleb128
from os import path
import json

class GameFrameworkSetting:
    __slots__ = [
        "fp", "settings", "header"
    ]

    def __init__(self, config_fp: str | None = None):
        if config_fp is None:
            self.fp = path.expanduser("~/AppData/LocalLow/创客火/LiteBeeClien/GameFrameworkSetting.dat")
        
        else:
            self.fp = config_fp
    
    @staticmethod
    def process_value(v: bytes):
        v = v.decode()

        try:
            if v.find('.') > -1:
                return float(v)
            else:
                return int(v)
            
        except ValueError:
            try:
                return json.loads(v)
            
            except json.decoder.JSONDecodeError:
                return v
        
    def read_settings(self) -> dict:
        output = list()
        
        with open(self.fp, "rb") as file:
            self.header = file.read(5)

            while char:=file.read(1):
                size = ord(char)
                
                if size > 127:
                    size = uleb128.to_int(char + file.read(1))
                
                output.append(file.read(size))
        
        self.settings = {
            output[i].decode(): self.process_value(output[i+1]) for i in range(0, len(output), 2)
        }
    
        return self.settings

    def write_settings(self):
        output = bytes()
        for key, value in self.settings.items():
            v = str(value).encode("utf-8")
            output += uleb128.from_int(len(key)) + key.encode("utf-8")\
                    + uleb128.from_int(len(v)) + v

        with open(self.fp, "wb") as file:
            file.write(
                self.header + output
            )

    def __getitem__(self, key: str):
        return self.settings[key]

    def __setitem__(self, key: str, value):
        self.settings[key] = value