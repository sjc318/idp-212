'''
  @file demo_get_distance.py
  @brief Get measurement data by PROXIMITY and DISTANCE hybrid mode.
  @note  TMF8801 only suport one mode, PROXIMITY and DISTANCE hybrid mode.
  @n 
  @n --------------------------------------------------------------------------------|
  @n |  Type     |   suport ranging mode     |  ranging ranges |  Accuracy           |
  @n |---------------------------------------|-----------------|---------------------|
  @n |  TMF8801  | PROXIMITY and DISTANCE    |                 |  20~100mm: +/-15mm  |
  @n |           |  hybrid mode(only one)    |    20~240cm     |  100~200mm: +/-10mm |
  @n |           |                           |                 |   >=200: +/-%5      |
  @n |---------------------------------------|-----------------|---------------------|
  @n |           |     PROXIMITY mode        |    0~10cm       |                     |
  @n |           |---------------------------|-----------------|   >=200: +/-%5      |
  @n |  TMF8701  |     DISTANCE mode         |    10~60cm      |  100~200mm: +/-10mm |
  @n |           |---------------------------|-----------------|                     | 
  @n |           | PROXIMITY and DISTANCE    |    0~60cm       |                     |
  @n |           |      hybrid mode          |                 |                     |
  @n |---------------------------------------|-----------------|----------------------
  @n 
  @n hardware conneted table:
  @n ------------------------------------------
  @n |  TMF8x01  |            MCU              |
  @n |-----------------------------------------|
  @n |    I2C    |       I2C Interface         |
  @n |-----------------------------------------|
  @n |    EN     |   not connected, floating   |
  @n |-----------------------------------------|
  @n |    INT    |   not connected, floating   |
  @n |-----------------------------------------|
  @n |    PIN0   |   not connected, floating   |
  @n |-----------------------------------------|
  @n |    PIN1   |    not connected, floating  |
  @n |-----------------------------------------|
  @n
  @Copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author [Arya](xue.peng@dfrobot.com)
  @version  V1.0
  @date  2021-04-06
  @url https://github.com/DFRobot/DFRobot_TMF8x01

'''
from utime import sleep
from machine import Pin, I2C

from libs.DFRobot_TMF8x01.DFRobot_TMF8x01 import DFRobot_TMF8701
import time

from libs.VL53L0X.VL53L0X import VL53L0X

def test_TMF8x01_get_distance():
    # Both options work
    i2c_bus = I2C(1, sda=Pin(10), scl=Pin(11), freq=100000)  # I2C0 on GP8 & GP9
    i2c_bus2 = I2C(0, sda=Pin(12), scl=Pin(13))
    #i2c_bus = I2C(id=1, sda=Pin(10), scl=Pin(11), freq=100000) # I2C0 on GP8 & GP9

    print(i2c_bus.scan(), i2c_bus2.scan()) # 65=0x41
    
    


    # Use the correct one - TODO can we auto detect this?
    tof = DFRobot_TMF8701(i2c_bus=i2c_bus)
    vl53l0 = VL53L0X(i2c_bus2)
    vl53l0.set_Vcsel_pulse_period(vl53l0.vcsel_period_type[0], 18)
    vl53l0.set_Vcsel_pulse_period(vl53l0.vcsel_period_type[1], 14)

    tof.start_measurement(calib_m = tof.eMODE_NO_CALIB, mode = tof.eCOMBINE)
    while True:
        print("Starting vl53l0...")
        '''
        # Start device
        vl53l0.start()

        # Read ten samples
        distance = vl53l0.read()
        #d2 = tof.get_distance_mm()
        print(f"Distance = {distance}mm")  # Check calibration!
        sleep(0.5)
        
        # Stop device
        vl53l0.stop()
        '''
        if(tof.is_data_ready() == True):
            d2 = tof.get_distance_mm()
            print(f"Distance 2 = {d2}mm")
'''
    while True:
        time.sleep(0.5)
        sensor.start()
        if(tof.is_data_ready() == True):
            print(f"Distance from left = {tof.get_distance_mm()} mm{" (For TMF8701, make sure you read about mode selection above!)" if device == "TMF8701" else ""}")
            dist = sensor.read()
            print("Distance from right:", dist, "mm")
        sensor.stop()
'''
      

if __name__ == "__main__":
    test_TMF8x01_get_distance()
'''

from machine import Pin, SoftI2C, I2C
from libs.tcs3472_micropython.tcs3472 import tcs3472
from utime import sleep

def test_tcs3472():
    # Both options works
    i2c_bus = SoftI2C(sda=Pin(8), scl=Pin(9), freq=100000)  # I2C0 on GP8 & GP9
    #i2c_bus = I2C(id=0, sda=Pin(8), scl=Pin(9)) # I2C0 on GP8 & GP9
    print(i2c_bus.scan())  # Get the address (nb 41=0x29)
    tcs = tcs3472(i2c_bus)

    while True:
        print("Light:", tcs.light())
        print("RGB:", tcs.rgb())
        sleep(1)

if __name__ == "__main__":
    test_tcs3472()
'''