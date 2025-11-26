import machine
import time
import random
from machine import Pin, PWM, I2C, SoftI2C
from libs.tcs3472_micropython.tcs3472 import tcs3472
from utime import sleep
import sys
import select

from libs.DFRobot_TMF8x01.DFRobot_TMF8x01 import DFRobot_TMF8701
from libs.VL53L0X.VL53L0X import VL53L0X

def fast_read_raw(i2c, addr=0x29):
    data = i2c.readfrom_mem(addr, 0x14 | 0x80, 8)  # COMMAND bit required!

    c = data[0] | (data[1] << 8)
    r = data[2] | (data[3] << 8)
    g = data[4] | (data[5] << 8)
    b = data[6] | (data[7] << 8)

    return c, r, g, b

if __name__ == "__main__":
    gp21 = Pin(21, Pin.OUT)
    gp21.value(1)
    time.sleep(2)
    i2c_bus = SoftI2C(sda=Pin(8), scl=Pin(9))
    tcs = tcs3472(i2c_bus)
    c, r, g, b = fast_read_raw(i2c_bus)
    gp21.value(0)
    time.sleep(1)
