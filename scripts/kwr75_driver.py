#!/usr/bin/python2

import rospy
import serial
import struct
from geometry_msgs.msg import Wrench

BYTE_SIZE = 8
STOP_BITS = 1
CONVERT_COMMAND = b'\x49\xAA\x0D\x0A'
class KWR75_ERROR:
    TIMEOUT = 0
    INVALID = 1

# Faster pyserial readline
# https://github.com/pyserial/pyserial/issues/216#issuecomment-369414522
class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s
    
    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)

            # Check if timed out
            if data == "":
                return bytearray()
            
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)

class KWR75Driver:
    def __init__(self, port, baudrate=460800, timeout=0.5):
        self.ser = serial.Serial(
            port=port, 
            baudrate=baudrate, 
            bytesize=BYTE_SIZE, 
            stopbits=STOP_BITS, 
            timeout=timeout, 
            write_timeout=timeout, 
            inter_byte_timeout=timeout
        )
        self.reader = ReadLine(self.ser)

    def read(self):
        # tell sensor to send conversion result
        self.ser.write(CONVERT_COMMAND)

        # receive sensor data
        data = self.reader.readline()

        if len(data) == 0:
            return None, KWR75_ERROR.TIMEOUT

        if len(data) != 28:
            return None, KWR75_ERROR.INVALID

        start = data[0:2]
        if start != b'\x49\xAA':
            return None, KWR75_ERROR.INVALID

        msg = Wrench()
        msg.force.x = struct.unpack('<f', data[2:6])[0]
        msg.force.y = struct.unpack('<f', data[6:10])[0]
        msg.force.z = struct.unpack('<f', data[10:14])[0]
        msg.torque.x = struct.unpack('<f', data[14:18])[0]
        msg.torque.y = struct.unpack('<f', data[18:22])[0]
        msg.torque.z = struct.unpack('<f', data[22:26])[0]

        return msg, None

def run():
    rospy.init_node('kwr75_driver')

    port = rospy.get_param('~port')
    baudrate = rospy.get_param('~baudrate', 460800)
    timeout = rospy.get_param('~timeout', 0.5)
    wrench_topic = rospy.get_param('~wrench_topic', 'kwr75/wrench')

    kwr75_driver = KWR75Driver(port, baudrate, timeout)
    wrench_pub = rospy.Publisher(wrench_topic, Wrench, queue_size=1)

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        wrench, err = kwr75_driver.read()

        if err == KWR75_ERROR.INVALID:
            pass
        elif err == KWR75_ERROR.TIMEOUT:
            rospy.logwarn('[kwr75_driver] Sensor Timed Out')
        else:
            wrench_pub.publish(wrench)
            rate.sleep()

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass