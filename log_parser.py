from typing import BinaryIO

READINGS = {
    1: {
        "name": "LowGData",
        "fields": [
            
        ]
    }
}

class LogParser:
    def __init__(self, input: BinaryIO):
        self.input = input

    def parse_one(self):
        discriminant = int.from_bytes(self.input.read(4), 'little')
        timestamp    = int.from_bytes(self.input.read(4), 'little')

    def __iter__(self):
