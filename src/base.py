from serial import Serial
from time import sleep
import json
from enum import Enum
from random import randint, choice

class JSONAble:
    def to_json(self):
        return json.dumps(self.__dict__)

class CameraFeed(str, Enum):
    Chassis = "Chassis"
    Cam0 = "Cam0"
    Cam1 = "Cam1"

class SomeData(JSONAble):
    def __init__(self, speed_l: int, speed_r: int, reverse: bool, camera: CameraFeed):
        self.speed_l = speed_l
        self.speed_r = speed_r
        self.reverse = reverse
        self.camera = camera

if __name__ == "__main__":

    port = Serial("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0", baudrate=1000000)
    print(port.name)

    while True:
        s = SomeData(randint(-128, 127), randint(-128, 127), choice((True, False)), choice((CameraFeed.Chassis, CameraFeed.Cam0, CameraFeed.Cam1)))
        # s = SomeData(0, 0, True, CameraFeed.Chassis)
        data = (json.dumps(s.__dict__) + '\0').encode("ascii")
        # data = ('oofasa' + '\0').encode("ascii")
        print(data)
        port.write(data)
        sleep(0.05)
        # exit()
