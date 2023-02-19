#!/bin/env python3
import rospy
from base import SendData, CameraFeed
from geometry_msgs.msg import Twist
from serial import Serial

class STMSerial():
    def __init__(self, port="/dev/serial/ttyACM0", baud=1000000):
        self.data = SendData(0, 0, False, CameraFeed.Chassis)
        self.port = Serial(port, baudrate=baud)
        print("Connected to:", self.port.name)

    def update_drive(self, drive_data):
        self.data.speed_l = drive_data.linear.x - drive_data.angular.z
        self.data.speed_r = drive_data.linear.x + drive_data.angular.z
        self.send()

    def send(self):
        self.port.write((str(self.data) + '\0').encode("ascii"))


rospy.init_node("jsonserial", anonymous=True)

s = STMSerial("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")

sub = rospy.Subscriber("/rover", Twist, callback=s.update_drive)
rospy.spin()
