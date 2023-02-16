from serial import Serial
from time import sleep
import json
from enum import Enum
from random import randint

class CameraFeed(str, Enum):
    Chassis = "Chassis"
    Cam0 = "Cam0"
    Cam1 = "Cam1"

class SomeData:
    def __init__(self, speed_l: int, speed_r: int, multiplier: int, reverse: bool, camera: CameraFeed):
    # def __init__(self, speed_l: int, speed_r: int, multiplier: int, reverse: bool):
        self.speed_l = speed_l
        self.speed_r = speed_r
        self.multiplier = multiplier
        self.reverse = reverse
        self.camera = camera

    def __iter__(self):
        yield from {
            "speed_l": self.speed_l,
            "speed_r": self.speed_r,
            "multiplier": self.multiplier,
            "reverse": self.reverse,
            "camera": self.camera
        }.items()

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=True, separators=(',', ':'))

    def __repr__(self):
        return self.__str__()

    def to_json(self):
        return self.__str__()

    @staticmethod
    def from_json(json_dict):
        return SomeData(json_dict['speed_l'], json_dict['speed_r'], json_dict['multiplier'], json_dict['reverse'], json_dict['camera'])
        # return SomeData(json_dict['speed_l'], json_dict['speed_r'], json_dict['multiplier'], json_dict['reverse'])

if __name__ == "__main__":

    port = Serial("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", baudrate=1000000)
    print(port.name)

    while True:
        s = SomeData(randint(-128, 127), 0, 0, True, CameraFeed.Chassis)
        print((str(s) + '\0').encode("ascii"))
        port.write((str(s) + '\0').encode("ascii"))
        sleep(1)

    # c = 0
    x = port.read_until(expected=b"\0")
    print(x)
    # while True:
    #     x = port.read_until(expected=b"\0")
    #     o = json.loads(x.decode()[:-1:])
    #     print(o, "\t count =", c, end='\r')
    #     c+=1
