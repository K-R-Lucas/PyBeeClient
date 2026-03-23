from litebee.utils import uleb128
from litebee.core import *
from litebee.commands import *


class Decompiler:
    def __init__(self, litebee_case_fp: str):
        self.file_path = litebee_case_fp
        self.buffer = open(self.file_path, "rb")
        self.token = dict()

    def read_uleb128(self):
        char = self.buffer.read(1)
        size = ord(char)

        if size > 127:
            size = uleb128.to_int(char + self.buffer.read(1))
    
    def read_string(self, size: int):
        return self.buffer.read(size).decode()

    def process_flag(self, flag: bytes) -> Command | None:
        match flag:
            case b"\x0A":
                uuid_size = self.read_uleb128()
                self.token["uuid"] = self.read_string(uuid_size)
            
            case b"\x12":
                name_size = self.read_uleb128()
                self.token["name"] = self.read_string(name_size)
            
            case b"\x18":
                self.token["gx"] = self.read_uleb128()
            
            case b"\x20":
                self.token["gy"] = self.read_uleb128()
            
            case b"\x2A":
                version_size = self.read_uleb128()
                self.token["version"] = self.read_string(version_size)
                

    def decompile_file(litebee_case_fp: str):
        with open(litebee_case_fp, "rb") as file:
            while char:=file.read(1):
                size = ord(char)
