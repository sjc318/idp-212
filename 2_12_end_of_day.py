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

leftPath = ("rc", "rc", "rc", "rc", "rc", "rc", "r", "ld", "rc", "rc", "rc", "rc", "rc", "rc")
rightPath = ("lc", "lc", "lc", "lc", "lc", "lc", "l", "ld", "rc", "rc", "rc", "rc", "rc", "rc")
#rightPath = ("s", "s", "s", "s", "s", "s", "180", "rc", "rc", "rc", "rc", "rc", "rc", "l", "l")

currentPath = ["l", "s", "r"]
#currentPath = ["r", "s", "l"]
currentPathIndex = 0
maxBoxesPerSide = 2

leftBoxesUnloaded = 0
rightBoxesUnloaded = 0

robotStatus = "returnToNode"
currentBoxColour = ""

decisionSkipTime = 0
ignoreBranchOnBayReturn = 0

timeToStopAcc = 0
accOn = False
accPos = "datum"

end = False
programOn = False
onLeft = True
#onLeft = False
onSecondFloor = False
trySecondFloor = False

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

def readTOF(tofs, readLeft):
    tofL = tofs[0]
    tofR = tofs[1]
    print(readLeft)
    if(readLeft == False):
        print("reading right")
        readingRight = tofR.read()
        return readingRight
    else:
        for i in range(100):
            if(tofL.is_data_ready() == True):
                readingLeft = tofL.get_distance_mm()
                print("left reading ", readingLeft)
                if(readingLeft > 0):
                    return readingLeft
            sleep(0.01)
    return 300

def turnLeft(sensors, motors):
    global accOn
    global timeToStopAcc
    print("turning left")
    motors[0].Reverse(speed = 60)
    motors[1].Forward(speed = 70)
    time.sleep(0.3)
    motors[0].off()
    readings = read_sensors(sensors)
    while(readings[1] == 0 or readings[2] == 0):
        if(accOn):
            if(checkForAcc(acc, timeToStopAcc)):
                accOn = False
        readings = read_sensors(sensors)
        time.sleep(0.005)
    #motors[1].off()
    return
    

def turnRight(sensors, motors):
    global accOn
    global timeToStopAcc
    print("turning right")
    motors[0].Forward(speed = 70)
    motors[1].Reverse(speed = 60)
    time.sleep(0.3)
    motors[1].off()
    readings = read_sensors(sensors)
    while(readings[1] == 0 or readings[2] == 0):
        if(accOn):
            if(checkForAcc(acc, timeToStopAcc)):
                accOn = False
        readings = read_sensors(sensors)
        time.sleep(0.005)
    #motors[0].off()
    return

def reverseLeft(sensors, motors):
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
    readings = read_sensors(sensors)
    while(readings[1] == 0 or readings[2] == 0):
        readings = read_sensors(sensors)
        time.sleep(0.01)
    motors[0].off()
    motors[1].off()

def reverseRight(sensors, motors):
    #motors[0].off()
    #motors[1].off()
    motors[0].Reverse(speed = 70)
    motors[1].Reverse(speed = 70)
    readings = read_sensors(sensors)
    for _ in range(20):
        if(readings[1] != readings[2]):
            maintainPath(readings, motors, reverse = True)
            readings = read_sensors(sensors)
        time.sleep(0.02)
    motors[1].off()
    time.sleep(1.5)
    readings = read_sensors(sensors)
    while(readings[1] == 0 or readings[2] == 0):
        readings = read_sensors(sensors)
        time.sleep(0.01)
    motors[0].off()
    motors[1].off()

def turn180(sensors, motors, clockwise = False, waitTime = 0):
    print("turning 180")
    while(waitTime > 0):
        waitTime -= 1
        readings = read_sensors(sensors)
        maintainPath(readings, motors)
        time.sleep(0.02)
    if(clockwise):
        motors[0].Forward(speed = 70)
        motors[1].Reverse(speed = 70)  
    else:
        motors[0].Reverse(speed = 70)
        motors[1].Forward(speed = 70)
    time.sleep(1.7)
    readings = read_sensors(sensors)
    while(readings[1] == 0 or readings[2] == 0):
        readings = read_sensors(sensors)
        time.sleep(0.01)
    motors[0].off()
    motors[1].off()

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
            motors[0].Forward(speed = 100)
        else:
            motors[1].Forward(speed = 100)
    else:
        if(readings[1] == False):
            motors[0].Reverse(speed = 80)
        else:
            motors[1].Reverse(speed = 80)

def checkForBox(isLeft, tofs):
    d = 0
    if(isLeft):
        d = readTOF(tofs, True)
    else:
        d = readTOF(tofs, False)
    return d

def fast_read_raw(i2c, addr=0x29):
    data = i2c.readfrom_mem(addr, 0x14 | 0x80, 8)  # COMMAND bit required!

    c = data[0] | (data[1] << 8)
    r = data[2] | (data[3] << 8)
    g = data[4] | (data[5] << 8)
    b = data[6] | (data[7] << 8)

    return c, r, g, b

def grabBox(acc):
    global leftBoxesUnloaded
    global rightBoxesUnloaded
    global onSecondFloor
    if(leftBoxesUnloaded == 2 and rightBoxesUnloaded == 2):
        onSecondFloor = True
    waitTime = setAccSpeed(acc, "datum")
    #print(time.time()-waitTime)
    while(checkForAcc(acc, waitTime) == False):
        time.sleep(0.01)
    gp21 = Pin(21, Pin.OUT)  #turning LED on
    gp21.value(1)
    i2c_bus = SoftI2C(sda=Pin(8), scl=Pin(9))
    time.sleep(0.1)
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
    gp21.value(0)
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
    global accOn
    global timeToStopAcc
    global leftBoxesUnloaded, rightBoxesUnloaded
    global onLeft
    waitTime = setAccSpeed(acc, "unloadPos")
    while checkForAcc(acc, waitTime) == False:
        time.sleep(0.001)
    motors[1].Reverse(speed = 70)
    motors[0].Reverse(speed = 70)
    time.sleep(1)
    '''
    timeToStopAcc = setAccSpeed(acc, "datum")
    accOn = True
    '''
    waitTime = setAccSpeed(acc, "datum")
    if(colour == "red"):
        turn180(sensors, motors, True)
    else:
        turn180(sensors, motors)
    if(onLeft):
        leftBoxesUnloaded+=1
    else:
        rightBoxesUnloaded+=1
    print("box count")
    print(leftBoxesUnloaded, rightBoxesUnloaded)
    while checkForAcc(acc, waitTime) == False:
        time.sleep(0.001)
    motors[0].Forward(speed = 85)
    motors[1].Forward(speed = 85)
    return

def bayPathCalculation(colour, onLeft, onSecondFloor):
    path = []
    if(onSecondFloor == True):
        if(colour == "red"):
            path += ["r", "r", "s", "s", "s", "s", "s", "s", "s"]
        elif(colour == "blue"):
            path += ["r", "l", "s", "s", "s", "s", "s", "s", "s"]
        elif(colour == "yellow"):
            path += ["r", "r", "s", "s", "s", "s", "s", "s", "r", "l"]
        elif(colour == "green"):
            path += ["r", "l", "s", "s", "s", "s", "s", "s", "l", "r"]#
    else:
        if(onLeft):
            if(colour == "red"):
                path += ["l", "s", "s", "s", "r"]
            elif(colour == "blue"):
                path += ["s"]
            elif(colour == "yellow"):
                path += ["l", "s", "s", "r"]
            elif(colour == "green"):
                path += ["l", "r"]
        else:
            if(colour == "red"):
                path += ["s"]
            elif(colour == "blue"):
                path += ["r", "s", "s", "s", "l"]
            elif(colour == "yellow"):
                path += ["r", "l"]
            elif(colour == "green"):
                path += ["r", "s", "s","l"]
    path.append("s")
    return path

def pathToNode(colour, leftBoxesUnloaded, rightBoxesUnloaded, maxBoxesPerSide, onSecondFloor):
    global onLeft
    global trySecondFloor
    path = []
    print("box count")
    print(leftBoxesUnloaded, rightBoxesUnloaded)
    if(colour in ("green", "blue") and leftBoxesUnloaded < maxBoxesPerSide):
        if(colour == "red"):
            path += ["l", "s", "s", "s", "r"]
        elif(colour == "blue"):
            path += ["s"]
        elif(colour == "yellow"):
            path += ["l", "s", "s", "r"]
        elif(colour == "green"):
            path += ["l", "r"]
        onLeft = True
    elif(leftBoxesUnloaded + rightBoxesUnloaded < maxBoxesPerSide * 2):
        if(colour == "red"):
            path += ["s"]
        elif(colour == "blue"):
            path += ["r", "s", "s", "s", "l"]
        elif(colour == "yellow"):
            path += ["r", "l"]
        elif(colour == "green"):
            path += ["r", "s", "s","l"]
        onLeft = False
    else:
        if(onSecondFloor == False and trySecondFloor == True):
            if(colour == "red"):
                path += ["s"]
                onLeft = False
            elif(colour == "blue"):
                path += ["s"]
                onLeft = True
            elif(colour == "yellow"):
                path += ["r", "l"]
                onLeft = False
            elif(colour == "green"):
                path += ["l", "r"]
                onLeft = True
        else:
            if(colour == "blue"):
                path = ["r", "s", "r"]
            elif(colour == "green"):
                path = ["r", "r"]
            elif(colour == "yellow"):
                path = ["l", "l"]
            elif(colour == "red"):
                path = ["l", "s", "l"]
            path.append("e")
    return path

def setAccSpeed(acc, desiredPos, desiredTime = 0.1):
    global accPos
    numPos0 = 0
    if(accPos == "datum"):
        numPos0 = 0
    elif(accPos == "intermediate"):
        numPos0 = 170
    elif(accPos == "carryPos"):
        numPos0 = 400
    elif(desiredPos == "topFloor"):
        numPos0 = 530
    elif(accPos == "unloadPos"):
        numPos0 = 610
    numPos1 = 0
    if(desiredPos == "datum"):
        numPos1 = -100
        accPos = "datum"
    elif(desiredPos == "intermediate"):
        numPos1 = 170
        accPos = "intermediate"
    elif(desiredPos == "carryPos"):
        numPos1 = 400
        accPos = "carryPos"
    elif(desiredPos == "topFloor"):
        numPos1 = 530
        accPos = "topFloor"
    elif(desiredPos == "unloadPos"):
        numPos1 = 610
        accPos = "unloadPos"
    posChange = numPos1 - numPos0
    speedRequired = posChange / desiredTime
    outputTime = desiredTime
    if(speedRequired > 100):
        speedRequired = 100
        outputTime = posChange / 100
        print("speed required too high")
    elif(speedRequired < -100):
        speedRequired = -100
        outputTime = posChange / -100
        print("speed required too low")
    if(speedRequired < 0):
        acc.set(dir = 1, speed = -speedRequired)
    else:
        acc.set(dir = 0, speed = speedRequired)
    return time.time() * 100000 + int(100000 * outputTime)
    
def checkForAcc(acc, timeToStop):
    #print(time.time() - timeToStop)
    if(time.time() * 100000 >= timeToStop):
        acc.set(dir = 0, speed = 0)
        return True
    return False

def turnDecision(branchProfile, sensors, motors, acc, path, pathIndex, tofs):
    global end
    global decisionSkipTime
    global robotStatus
    global currentBoxColour
    global ignoreBranchOnBayReturn
    global currentPath
    global leftBoxesUnloaded, rightBoxesUnloaded, maxBoxesPerSide, onLeft
    global accOn
    global timeToStopAcc

    straightTime = 15
    confirmationTime = 0.06
    if(robotStatus == "returnToNode" and path[pathIndex] == "e"):
                motors[0].Forward(speed = 70)
                motors[1].Forward(speed = 70)
                time.sleep(2)
                motors[0].off()
                motors[1].off()
                end = True
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
            if(robotStatus == "goToBay" and pathIndex == len(path) - 1):
                motors[0].off()
                motors[1].off()
                unloadBox(acc, sensors, motors, currentBoxColour)
                robotStatus = "returnToNode"
                currentPath = pathToNode(currentBoxColour, leftBoxesUnloaded, rightBoxesUnloaded, maxBoxesPerSide, onSecondFloor)
                return 0
            return pathIndex
        elif path[pathIndex] == "ld":
            timeToStopAcc = setAccSpeed(acc, "topFloor", 5)
            accOn = True
            turnLeft(sensors, motors)
            return pathIndex + 1
        else:
            return pathIndex

    elif branchProfile == [True, True, True]:
        if(robotStatus == "goToBay"):
            if(pathIndex == len(path) - 1):
                motors[0].off()
                motors[1].off()
                unloadBox(acc, sensors, motors, currentBoxColour)
                robotStatus = "returnToNode"
                currentPath = pathToNode(currentBoxColour, leftBoxesUnloaded, rightBoxesUnloaded, maxBoxesPerSide, onSecondFloor)
                return 0
            currentPath = bayPathCalculation(currentBoxColour, onLeft, onSecondFloor)
            return pathIndex
            decisionSkipTime = straightTime
            return pathIndex
        elif(robotStatus == "followCircuit"):
            timeToStopAcc = setAccSpeed(acc, "datum")
            accOn = True
        decisionSkipTime = straightTime
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
        if(ignoreBranchOnBayReturn > 0):
            return pathIndex
        if updatedBranchProfile == [True, True, False]:
            print("Left branch")
            print(pathIndex, path[pathIndex])
            if path[pathIndex] == "l":
                turnLeft(sensors, motors)
            elif path[pathIndex] == "s":
                print("Going straight")
                decisionSkipTime = straightTime
            elif path[pathIndex] == "lc":
                motors[0].off()
                motors[1].off()
                time.sleep(0.1)
                if(readTOF(tofs, True) < 250):
                    turnLeft(sensors, motors)
                    robotStatus = "pickupBox"
                    decisionSkipTime = straightTime
                else:
                    print("Going straight")
                    decisionSkipTime = straightTime
            return pathIndex + 1
        return pathIndex

    elif branchProfile == [False, True, True]:
        if(ignoreBranchOnBayReturn > 0):
            return pathIndex
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
    global robotStatus
    global currentBoxColour
    global ignoreBranchOnBayReturn
    global accOn
    global timeToStopAcc
    global leftBoxesUnloaded, rightBoxesUnloaded, maxBoxesPerSide, onLeft
    global currentPathIndex
    global currentPath
    global onSecondFloor
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
                    if(accOn):
                        if(checkForAcc(acc, timeToStopAcc)):
                            accOn = False
                    readings = read_sensors(sensors)
                    #TOFreadings = readTOF(tofs)
                    #TOFLList.append(TOFreadings[0])
                    #TOFRList.append(TOFreadings[1])
                    #print(count, TOFreadings[1])
                    ##motors[0].off()
                    ##motors[1].off()
                    if(robotStatus == "followCircuit"):
                        motors[0].Forward(speed = 85)
                        motors[1].Forward(speed = 85)
                        '''
                        if(accOn == False):
                            timeToStopAcc = setAccSpeed(acc, "intermediate", 3)
                            accOn = True
                        '''
                        if(readings[1] != readings[2]):
                            maintainPath(readings, motors)
                        branchProfile = turnDetector(readings)
                        if(decisionSkipTime == 0):
                            if(onLeft):
                                currentPathIndex = turnDecision(branchProfile, sensors, motors, acc, leftPath, currentPathIndex, tofs)
                            else:
                                currentPathIndex = turnDecision(branchProfile, sensors, motors, acc, rightPath, currentPathIndex, tofs)
                        elif(decisionSkipTime > 0):
                            #print(decisionSkipTime)
                            decisionSkipTime -= 1
                        else:
                            decisionSkipTime = 0
                        #count+=1
                        time.sleep(0.01)

                    elif(robotStatus == "pickupBox"):
                        branchProfile = turnDetector(readings)
                        if(branchProfile != [False, False, False]):
                            motors[0].Forward(speed = 40)
                            motors[1].Forward(speed = 40)          
                            if(readings[1] != readings[2]):
                                maintainPath(readings, motors)
                        else:
                            motors[0].Forward(speed = 40)
                            motors[1].Forward(speed = 40) 
                            time.sleep(0.15)
                            motors[0].off()
                            motors[1].off()
                            currentBoxColour = grabBox(acc)
                            if(onLeft or onSecondFloor):
                                reverseLeft(sensors, motors)
                            else:
                                reverseRight(sensors, motors)
                            if(onSecondFloor == False):
                                waitTime = setAccSpeed(acc, "carryPos")
                                while checkForAcc(acc, waitTime) == False:
                                    time.sleep(0.001)
                            robotStatus = "goToBay"
                            currentPath = bayPathCalculation(currentBoxColour, onLeft, onSecondFloor)
                            currentPathIndex = 0
                            ignoreBranchOnBayReturn = 200
                        time.sleep(0.01)
                
                    elif(robotStatus == "goToBay"):
                        motors[0].Forward(speed = 85)
                        motors[1].Forward(speed = 85)
                        if(readings[1] != readings[2]):
                            maintainPath(readings, motors)
                        branchProfile = turnDetector(readings)
                        #print(branchProfile)
                        if(ignoreBranchOnBayReturn > 0):
                            ignoreBranchOnBayReturn -= 1
                            #print(ignoreBranchOnBayReturn)
                        if(decisionSkipTime == 0):
                            currentPathIndex = turnDecision(branchProfile, sensors, motors, acc, currentPath, currentPathIndex, tofs)
                        elif(decisionSkipTime > 0):
                            print(decisionSkipTime)
                            decisionSkipTime -= 1
                        else:
                            decisionSkipTime = 0
                        time.sleep(0.01)

                    elif(robotStatus == "returnToNode"):
                        #print("returning")
                        motors[0].Forward(speed = 85)
                        motors[1].Forward(speed = 85)          
                        if(readings[1] != readings[2]):
                            maintainPath(readings, motors)
                        branchProfile = turnDetector(readings)
                        #print(branchProfile)
                        if(decisionSkipTime == 0):
                            currentPathIndex = turnDecision(branchProfile, sensors, motors, acc, currentPath, currentPathIndex, tofs)
                            if(currentPathIndex == len(currentPath)):
                                currentPathIndex = 0
                                robotStatus = "followCircuit"
                                timeToStopAcc = setAccSpeed(acc, "intermediate", 4)
                                accOn = True
                        elif(decisionSkipTime > 0):
                            print(decisionSkipTime)
                            decisionSkipTime -= 1
                        else:
                            decisionSkipTime = 0
                        time.sleep(0.01)
            else:
                #TOFreadings = readTOF(tofs, True)
                #print(TOFreadings, time.time())
                acc.set(dir = 1, speed = 100)
                if(read_sensor(button) == 1):
                    yellowLED.value(1)
                    programOn = True
    except KeyboardInterrupt:
        print("exit")
        motors[0].off()
        motors[1].off()

if __name__ == "__main__":
    sensors = []
    sensors.append(setup_sensor(28)) #blue
    sensors.append(setup_sensor(26)) #purple
    sensors.append(setup_sensor(22)) #gray
    sensors.append(setup_sensor(27)) #brown
    button = setup_sensor(20)
    motors = []
    motors.append(setup_motor(4, 5))
    motors.append(setup_motor(7, 6))
    acc = setup_actuator(3, 2)
    yellowLED = Pin(14, Pin.OUT)
    yellowLED.value(0)
    
    pin1 = Pin(21, Pin.OUT)
    pin1.value(1)
    i2c_bus = SoftI2C(sda=Pin(8), scl=Pin(9))
    time.sleep(0.5)
    tcs = tcs3472(i2c_bus)
    time.sleep(0.5)
    pin1.value(0)

    mainLoop(sensors, button, motors, acc, yellowLED)
