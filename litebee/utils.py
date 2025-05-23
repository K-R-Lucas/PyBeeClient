class uleb128:
    @staticmethod
    def from_int(input_int: float):
        assert input_int >= 0
        
        byte_list = []
        value = int(input_int)

        while True:
            byte = 0b01111111 & value
            value >>= 7

            if value != 0:
                byte |= 0b10000000
            
            byte_list.append(byte)

            if value == 0:
                break
        
        return bytes(byte_list)

    @staticmethod
    def to_int(input_bytes: bytes):
        result = 0
        shift = 0

        for byte in input_bytes:
            result |= (0b01111111 & byte) << shift

            if not (0b10000000 & byte):
                break

            shift += 7

        return result