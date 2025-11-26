import machine
#from gpiozero import DigitalInputDevice
import time
import random
#from turn_detector.py import turnDetector
from machine import Pin, PWM, I2C, SoftI2C
from libs.tcs3472_micropython.tcs3472 import tcs3472
from utime import sleep
import sysimport machine
#from gpiozero import DigitalInputDevice
import time
import random
#from turn_detector.py import turnDetector
from machine import Pin, PWM, I2C, SoftI2C
from libs.tcs3472_micropython.tcs3472 import tcs3472
from utime import sleep
import sys
import select

from libs.DFRobot_TMF8x01.DFRobot_TMF8x01 import DFRobot_TMF8701
from libs.VL53L0X.VL53L0X import VL53L0X

circuitPath = ["l","s","r","rc","rc","rc","rc","rc","rc","rc","rc","rc","rc","s","s","s","r","s","l","s"] # left, right or straight
circuitPathIndex = 0
bayPath = []
bayPathIndex = 0
returnPath = []
returnPathIndex = 0
robotStatus = "followCircuit"
#robotStatus = "returnToCircuit"
decisionSkipTime = 0
end = False
programOn = False
TOFLList = []
TOFRList = []
branchList = []
count = 0
frontDistance = 200
currentBoxColour = ""
boxCount = 0
# pid parameters below, to be tuned
_SENSOR_WEIGHTS = [-3.5, -1, 1, 3.5]  # weights for left to right sensors based on position
pid = PID(Kp=25.0, Ki=0.0, Kd=0.0, output_limits=(-100, 100), integral_limit=100)
'''
1. set ki = 0, kd = 0, increase kp until oscillations start, then reduce kp to about half that value
2. increase kd until oscillations damp quickly
3. increase ki slowly to eliminate steady-state error start from 0.1
4. lower base_speed while tuning
'''
last_error = 0.0



class Actuator:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)  # set motor direction pin
        self.pwm = PWM(Pin(PWMPin))  # set motor pwm pin
        self.pwm.freq(1000)  # set PWM frequency
        self.pwm.duty_u16(0)  # set duty cycle - 0=off
           
    def set(self, dir, speed):
        self.mDir.value(dir)                     # forward = 0 reverse = 1 motor
        self.pwm.duty_u16(int(65535 * speed / 100))  # speed range 0-100 motor

class Motor:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)  # set motor direction pin
        self.pwm = PWM(PWMPin)            # set motor pwm pin
        self.pwm.freq(1000)               # set PWM frequency
        self.pwm.duty_u16(0)              # set duty cycle - 0=off

    def off(self):
        self.pwm.duty_u16(0)

    def Forward(self, speed=100):
        self.mDir.value(0)                # forward = 0 reverse = 1 motor
        self.pwm.duty_u16(int(65535 * speed / 100))  # speed range 0-100 motor

    def Reverse(self, speed=30):
        self.mDir.value(1)
        self.pwm.duty_u16(int(65535 * speed / 100))

class PID:
    def __init__(self, Kp, Ki, Kd, output_limits=(None, None), integral_limit=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.min_out, self.max_out = output_limits
        self.int_limit = integral_limit

        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def reset(self):
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def compute(self, ref, measure):
        now = time.ticks_ms() / 1000.0  # current time in seconds
        error = ref - measure

        if self.last_time is None:
            dt = 0.02 # tbd
        else:
            dt = now - self.last_time
            if dt <= 0.0:
                dt = 0.001
        
        self.integral += error * dt
        if self.int_limit is not None:
            self.integral = max(min(self.integral, self.int_limit), -self.int_limit)
        
        derivative = (error - self.previous_error) / dt
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        if self.max_out is not None:
            output = min(output, self.max_out)
        if self.min_out is not None:
            output = max(output, self.min_out)
        
        self.previous_error = error
        self.last_time = now

        return output

def setup_actuator(dPin, pPin):
    acc = Actuator(dPin, pPin)
    return acc
        
def setup_motor(dPin, pPin):
    motor = Motor(dirPin=dPin, PWMPin=pPin)
    return motor

def setup_sensor(pin_num):
    """Initialize a digital input pin for a sensor."""
    return machine.Pin(pin_num, machine.Pin.IN)
    # If needed, enable internal pull-ups/pull-downs:
    # return machine.Pin(pin_num, machine.Pin.IN, machine.Pin.PULL_UP)

def read_sensor(pin):
    """Return True if surface is white (HIGH), False if black (LOW)."""
    return bool(pin.value())

def readTOF(tofs):
    tofL = tofs[0]
    tofR = tofs[1]
    '''
    for i in range(100):
        if(tofL.is_data_ready() == True):
            readingLeft = tofL.get_distance_mm()
            if(readingLeft > 0):
                break
        sleep(0.01)
    '''
    readingLeft = 0
    #readingLeft = tofL.get_distance_mm()
    readingRight = tofR.read()
    return [readingLeft, readingRight]

def turnLeft(sensors, motors):
    print("turning left")
    motors[0].off()
    motors[1].off()
    motors[0].Reverse(speed = 50)
    motors[1].Forward(speed = 70)
    time.sleep(0.4)
    motors[0].off()
    readings = read_sensors(sensors)
    while(readings[1] == 0 or readings[2] == 0):
        readings = read_sensors(sensors)
        time.sleep(0.01)
    motors[1].off()
    return
    
def turnRight(sensors, motors):
    print("turning right")
    motors[0].off()
    motors[1].off()
    motors[0].Forward(speed = 70)
    motors[1].Reverse(speed = 50)
    time.sleep(0.4)
    motors[1].off()
    readings = read_sensors(sensors)
    while(readings[1] == 0 or readings[2] == 0):
        readings = read_sensors(sensors)
        time.sleep(0.01)
    motors[0].off()
    return

def reverseLeft(sensors, motors):
    motors[0].off()
    motors[1].off()
    motors[0].Reverse(speed = 70)
    motors[1].Reverse(speed = 70)
    readings = read_sensors(sensors)
    for _ in range(40):
        if(readings[1] != readings[2]):
            maintainPath(readings, motors, reverse = True)
            readings = read_sensors(sensors)
        time.sleep(0.02)
    motors[0].off()
    time.sleep(1.5)
    #motors[0].Forward(30)
    readings = read_sensors(sensors)
    while(readings[1] == 0 or readings[2] == 0):
        readings = read_sensors(sensors)
        time.sleep(0.01)
    '''
    while(readings[2] == 1):
        readings = read_sensors(sensors)
        time.sleep(0.01)
    '''
    motors[0].off()
    motors[1].off()
    
def turn180(sensors, motors, clockwise = False):
    print("turning 180")
    motors[1].Reverse(speed = 70)
    motors[0].Reverse(speed = 70)
    time.sleep(1)
    if(clockwise):
        motors[0].Forward(speed = 70)
    else:
        motors[1].Forward(speed = 70)
    time.sleep(1)
    readings = read_sensors(sensors)
    while(readings[1] == 0 or readings[2] == 0):
        readings = read_sensors(sensors)
        time.sleep(0.01)
    motors[0].off()
    motors[1].off()
    return

def turnDetector(sensorReadings):
    output = [False,False,False]
    if(sensorReadings[0] == 1):
        #left branch detected
        output[0] = True
    if(sensorReadings[1] == 1 or sensorReadings[2] == 1):
        #top branch detected
        output[1] = True
    if(sensorReadings[3] == 1):
        #right branch detected
        output[2] = True
    
    return output

def maintainPath(readings, motors, reverse = False):
    if(reverse==False):
        if(readings[1] == False):
            motors[0].off()
            motors[0].Forward(speed = 85)
            #print("motorL speed increased")
        else:
            motors[1].off()
            motors[1].Forward(speed = 85)
            #print("motorR speed increased")
    else:
        if(readings[1] == False):
            motors[0].off()
            motors[0].Reverse(speed = 80)
        else:
            motors[1].off()
            motors[1].Reverse(speed = 80)

def maintainPathPID(readings, motors, reverse = False):
    global last_error

    active = [1 if (not r) else 0 for r in readings]

    sum = 0.0
    active_count = 0
    for w, a in zip(_SENSOR_WEIGHTS, active):
        sum += w * a
        active_count += a
    
    if active_count > 0:
        error = sum / active_count
        last_error = error
    else:
        error = last_error
        base_speed = max(50, base_speed - 20)
    
    correction = pid.compute(0.0, error)

    if reverse:
        correction = -correction
    
    left_speed = base_speed - correction
    right_speed = base_speed + correction

    def clamp(x, min=0, max=100):
        return max(min(x, max), min)
    
    left_speed = clamp(left_speed)
    right_speed = clamp(right_speed)

    if not reverse:
        motors[0].off()
        motors[0].Forward(speed = left_speed)
        motors[1].off()
        motors[1].Forward(speed = right_speed)
    else:
        motors[0].off()
        motors[0].Reverse(speed = left_speed)
        motors[1].off()
        motors[1].Reverse(speed = right_speed)

def checkForBox(isLeft, tofs):
    d = readTOF(tofs)
    print(d[1])
    return d[1]

def fast_read_raw(i2c, addr=0x29):
    data = i2c.readfrom_mem(addr, 0x14 | 0x80, 8)  # COMMAND bit required!

    c = data[0] | (data[1] << 8)
    r = data[2] | (data[3] << 8)
    g = data[4] | (data[5] << 8)
    b = data[6] | (data[7] << 8)

    return c, r, g, b

def grabBox(acc):
    acc.set(dir = 1, speed=100)
    time.sleep(3)
    acc.set(dir = 0, speed=0)
    gp21 = Pin(21, Pin.OUT)  #turning LED on
    gp21.value(1)
    i2c_bus = SoftI2C(sda=Pin(8), scl=Pin(9))
    tcs = tcs3472(i2c_bus)
    # MUST initialize the sensor
    r_sum = g_sum = b_sum = 0
    total = 1000
    print("starting")
    for _ in range(total):
        c, r, g, b = fast_read_raw(i2c_bus)
        r_sum += r
        g_sum += g
        b_sum += b

    total_rgb = r_sum + g_sum + b_sum
    print("done")
    if total_rgb == 0:
        return (0, 0, 0)   
    r_ratio=r_sum / total_rgb
    g_ratio=g_sum / total_rgb
    b_ratio=b_sum / total_rgb
    
    print(r_ratio,g_ratio,b_ratio)
    if b_ratio>0.55 and r_ratio<0.15:   #checking for red
        print("blue")
        return ("blue")
    elif g_ratio>0.40 and r_ratio>0.25 and b_ratio<0.30:      #checking for blue
        print("yellow")
        return("yellow")
    elif r_ratio>0.30 and g_ratio<0.30 and b_ratio<0.40:  #checking for green
        print("red")
        return("red")
    elif g_ratio>0.30 and b_ratio>0.35 and r_ratio<0.30:   #checking for yellow
        print("green")
        return("green")
    else:
        print("unkown")
        return("red")

def unloadBox(acc, sensors, motors, colour):
    acc.set(dir = 0, speed=50)
    time.sleep(11)
    acc.set(dir = 0, speed=0)
    if(colour == "red"):
        turn180(sensors, motors, True)
    else:
        turn180(sensors, motors)
    acc.set(dir = 1, speed=100)
    time.sleep(6)
    acc.set(dir = 0, speed=0)
    time.sleep(0.5)
    acc.set(dir = 0, speed=50)
    time.sleep(3.6)
    acc.set(dir = 0, speed=0)
    return

def reversePath(Path):
    reversed_path = Path[::-1]
    result = []

    for instruction in reversed_path:
        if instruction == "l":
            result.append("r")
        elif instruction == "r":
            result.append("l")
        elif instruction in ("lc", "rc"):
            result.append("s")
        else:
            result.append("s")
    return result

def bayPathCalculation(currentBoxColour, path, pathIndex):
    pathTaken = path[:pathIndex]
    newPath = []
    newPath.append("s")
    #need to splice the front of the list so that it returns to the first branch before the bays
    #this is currently excluding the first three indices
    pathTaken = pathTaken[3:-1]
    newPath += reversePath(pathTaken)
    if(currentBoxColour == "red"):
        newPath += ["l", "s", "s", "s", "r"]
    elif(currentBoxColour == "blue"):
        newPath += ["s"]
    elif(currentBoxColour == "yellow"):
        newPath += ["l", "s", "s", "r"]
    elif(currentBoxColour == "green"):
        newPath += ["l", "r"]
    else:
        print("Colour not recognised")
    newPath += ["s"]
    return newPath

def returnPathCalculation(path):
    returnPath = reversePath(path)
    returnPath = returnPath[1:]
    return returnPath

def homePathCalculation(colour):
    path = []
    if(colour == "blue"):
        path = ["r", "s", "r"]
    elif(colour == "green"):
        path = ["r", "r"]
    elif(colour == "yellow"):
        path = ["l", "l"]
    elif(colour == "red"):
        path = ["l", "r", "l"]
    path.append("s")
    return path

        
def turnDecision(branchProfile, sensors, motors, acc, path, pathIndex, tofs):
    global end
    global decisionSkipTime
    global robotStatus
    global returnPath
    global returnPathIndex
    global currentBoxColour
    straightTime = 8
    confirmationTime = 0.06
    if branchProfile == [False, True, False]:
        return pathIndex
    elif branchProfile == [False, False, False]:
        return pathIndex
    elif branchProfile == [True, False, True]:
        print("T")
        print(pathIndex, path[pathIndex])
        if path[pathIndex] == "r":
            turnRight(sensors, motors)
            return pathIndex + 1
        elif path[pathIndex] == "l":
            turnLeft(sensors, motors)
            return pathIndex + 1
        elif path[pathIndex] == "s":
            if(robotStatus == "returnToCircuit"):
                motors[0].Forward(speed = 70)
                motors[1].Forward(speed = 70)
                time.sleep(1)
                motors[0].off()
                motors[1].off()
                end = True
            elif(robotStatus == "goToBay" and pathIndex == len(path) - 1):
                motors[0].off()
                motors[1].off()
                unloadBox(acc, sensors, motors, currentBoxColour)
                robotStatus = "returnToCircuit"
                returnPathIndex = 0
                if(boxCount == 4):
                    #return home
                    returnPath = homePathCalculation(currentBoxColour)
                else:
                    returnPath = returnPathCalculation(bayPath)
                print(returnPath) 
            return pathIndex
        else:
            return pathIndex

    elif branchProfile == [True, True, True]:
        if(robotStatus == "goToBay" and pathIndex == len(path) - 1):
            motors[0].off()
            motors[1].off()
            unloadBox(acc, sensors, motors, currentBoxColour)
            boxCount += 1
            robotStatus = "returnToCircuit"
            returnPathIndex = 0
            if(boxCount == 4):
                #return home
                returnPath = homePathCalculation(currentBoxColour)
            else:
                returnPath = returnPathCalculation(bayPath)
            print(returnPath)
            return pathIndex
        decisionSkipTime = straightTime
        print(TOFLList)
        print(TOFRList)
        print(branchList)
        return pathIndex
    
    time.sleep(confirmationTime)
    readings = read_sensors(sensors)
    updatedBranchProfile = turnDetector(readings)
    if branchProfile == [True, False, False]:
        if updatedBranchProfile == [True, False, False]:
            turnLeft(sensors, motors)
        return pathIndex

    elif branchProfile == [False, False, True]:
        if updatedBranchProfile == [False, False, True]:
            turnRight(sensors, motors)
        return pathIndex

    elif branchProfile == [True, True, False]:
        if updatedBranchProfile == [True, True, False]:
            print("Left branch")
            print(pathIndex, path[pathIndex])
            if path[pathIndex] == "l":
                turnLeft(sensors, motors)
            elif path[pathIndex] == "s":
                print("Going straight")
                decisionSkipTime = straightTime
            elif path[pathIndex] == "lc":
                if(checkForBox(True, tofs)):
                    #turnLeft()
                    #robotStatus = "pickupBox"
                    decisionSkipTime = straightTime
                else:
                    print("Going straight")
                    decisionSkipTime = straightTime
            return pathIndex + 1
        return pathIndex

    elif branchProfile == [False, True, True]:
        if updatedBranchProfile == [False, True, True]:
            print("Right branch")
            print(pathIndex, path[pathIndex])
            if path[pathIndex] == "r":
                turnRight(sensors, motors)
            elif path[pathIndex] == "s":
                print("Going straight")
                decisionSkipTime = straightTime
            elif path[pathIndex] == "rc":
                if(checkForBox(False, tofs) < 300):
                    turnRight(sensors, motors)
                    robotStatus = "pickupBox"
                    decisionSkipTime = straightTime
                else:
                    print("Going straight")
                    decisionSkipTime = straightTime
            return pathIndex + 1
        return pathIndex

    else:
        return pathIndex


def read_sensors(sensors):
    # Simulate line detection
    # 0 = line detected (black), 1 = white ground
    readings = []
    for s in sensors:
        readings.append(read_sensor(s))
    return readings

def key_pressed():
    # Non-blocking key press check
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    return bool(dr)

def mainLoop(sensors, button, motors, acc, yellowLED):
    global decisionSkipTime
    global programOn
    global circuitPathIndex
    global TOFLList
    global TOFRList
    global count
    global robotStatus
    global bayPathIndex
    global bayPath
    global frontDistance
    global returnPath
    global returnPathIndex
    global currentBoxColour
    i2c_busL = I2C(1, sda=Pin(10), scl=Pin(11), freq = 100000)  # I2C0 on GP8 & GP9
    i2c_busR = I2C(0, sda=Pin(12), scl=Pin(13), freq = 100000)
    
    print(i2c_busL.scan(), i2c_busR.scan())
    
    tofL = DFRobot_TMF8701(i2c_bus=i2c_busL)
    while(tofL.begin() != 0):
        sleep(0.5)
    tofR = VL53L0X(i2c_busR)
    tofR.set_Vcsel_pulse_period(tofR.vcsel_period_type[0], 18)
    tofR.set_Vcsel_pulse_period(tofR.vcsel_period_type[1], 14)

    tofL.start_measurement(calib_m = tofL.eMODE_NO_CALIB, mode = tofL.eCOMBINE)
    tofR.start()
    
    tofs = [tofL, tofR]
    
    print("ready")
    
    try:
        while True:
            if(programOn):
                if(end):
                    yellowLED.value(0)
                else:
                    readings = read_sensors(sensors)
                    #TOFreadings = readTOF(tofs)
                    #TOFLList.append(TOFreadings[0])
                    #TOFRList.append(TOFreadings[1])
                    #print(count, TOFreadings[1])
                    #motors[0].off()
                    #motors[1].off()
                    if(robotStatus == "followCircuit"):
                        motors[0].Forward(speed = 70)
                        motors[1].Forward(speed = 70)          
                        if(readings[1] != readings[2]):
                            maintainPath(readings, motors)
                        branchProfile = turnDetector(readings)
                        if branchProfile == [True, True, False] or branchProfile == [False, True, True]:
                            branchList.append(1000)
                        else:
                            branchList.append(0)
                        #print(branchProfile)
                        if(decisionSkipTime == 0):
                            circuitPathIndex = turnDecision(branchProfile, sensors, motors, acc, circuitPath, circuitPathIndex, tofs)
                        elif(decisionSkipTime > 0):
                            #print(decisionSkipTime)
                            decisionSkipTime -= 1
                        else:
                            decisionSkipTime = 0
                        #print("on path")
                        #print(readings, turnDetector(readings))
                        #print(listIndex)
                        count+=1
                        time.sleep(0.02)

                    elif(robotStatus == "pickupBox"):
                        branchProfile = turnDetector(readings)
                        if(branchProfile != [False, False, False]):
                            motors[0].Forward(speed = 30)
                            motors[1].Forward(speed = 30)          
                            if(readings[1] != readings[2]):
                                maintainPath(readings, motors)
                        else:
                            motors[0].Forward(speed = 30)
                            motors[1].Forward(speed = 30) 
                            time.sleep(0.2)
                            motors[0].off()
                            motors[1].off()
                            currentBoxColour = grabBox(acc)
                            reverseLeft(sensors, motors)
                            robotStatus = "goToBay"
                            bayPathIndex = 0
                            bayPath = bayPathCalculation(currentBoxColour, circuitPath, circuitPathIndex)
                        time.sleep(0.02)
                
                    elif(robotStatus == "goToBay"):
                        motors[0].Forward(speed = 70)
                        motors[1].Forward(speed = 70)
                        if(readings[1] != readings[2]):
                            maintainPath(readings, motors)
                        branchProfile = turnDetector(readings)
                        #print(branchProfile)
                        if(decisionSkipTime == 0):
                            bayPathIndex = turnDecision(branchProfile, sensors, motors, acc, bayPath, bayPathIndex, tofs)
                        elif(decisionSkipTime > 0):
                            print(decisionSkipTime)
                            decisionSkipTime -= 1
                        else:
                            decisionSkipTime = 0
                        time.sleep(0.02)

                    elif(robotStatus == "returnToCircuit"):
                        #print("returning")
                        motors[0].Forward(speed = 70)
                        motors[1].Forward(speed = 70)          
                        if(readings[1] != readings[2]):
                            maintainPath(readings, motors)
                        branchProfile = turnDetector(readings)
                        #print(branchProfile)
                        if(decisionSkipTime == 0):
                            returnPathIndex = turnDecision(branchProfile, sensors, motors, acc, returnPath, returnPathIndex, tofs)
                            if(returnPathIndex == len(returnPath)):
                                robotStatus = "followCircuit"
                        elif(decisionSkipTime > 0):
                            print(decisionSkipTime)
                            decisionSkipTime -= 1
                        else:
                            decisionSkipTime = 0
                        time.sleep(0.02)
            else:
                if(read_sensor(button) == 1):
                    yellowLED.value(1)
                    programOn = True
                    acc.set(dir = 0, speed=50)
                    time.sleep(3.6)
                    acc.set(dir = 0, speed=0)
    except KeyboardInterrupt:
        motors[0].off()
        motors[1].off()

if __name__ == "__main__":
    sensors = []
    sensors.append(setup_sensor(28)) #blue, far left
    sensors.append(setup_sensor(26)) #purple, middle left
    sensors.append(setup_sensor(22)) #gray, middle right
    sensors.append(setup_sensor(27)) #brown, far right
    button = setup_sensor(20)
    motors = []
    motors.append(setup_motor(4, 5))
    motors.append(setup_motor(7, 6))
    acc = setup_actuator(3, 2)
    yellowLED = Pin(14, Pin.OUT)
    yellowLED.value(0)

    pin1 = Pin(21, Pin.OUT) # red LED control
    pin1.value(0)

    pin3 = Pin(19, Pin.OUT) # ultrasonic sensor
    pin3.value(0)

    pin4 = Pin(17, Pin.OUT) # crash sensor
    pin4.value(0)

    pin5 = Pin(16, Pin.OUT) # crash sensor
    pin5.value(0)

    pin7 = Pin(13, Pin.OUT) # VL53L0 scl
    pin7.value(0)

    pin8 = Pin(12, Pin.OUT) # VL53L0 sda
    pin8.value(0)

    pin9 = Pin(11, Pin.OUT) # TMF8701 scl
    pin9.value(0)

    pin10 = Pin(10, Pin.OUT) # TMF8701 sda
    pin10.value(0)

    pin11 = Pin(9, Pin.OUT) # color sensor scl
    pin11.value(0)

    pin12 = Pin(8, Pin.OUT) # color sensor sda
    pin12.value(0)

    mainLoop(sensors, button, motors, acc, yellowLED)