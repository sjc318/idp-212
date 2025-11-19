import machine
#from gpiozero import DigitalInputDevice
import time
import random
#from turn_detector.py import turnDetector
from machine import Pin, PWM
from utime import sleep
import sys
import select

circuitPath = ["l","s","r","s","s","s","s","s","s","s","s","s","s","s","s","s","r","s","l","s"] # left, right or straight
circuitPathIndex = 0
robotStatus = "followCircuit"
decisionSkipTime = 0
end = False
programOn = False

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

def turnLeft(sensors, motors):
    print("turning left")
    motors[0].off()
    motors[1].off()
    motors[1].Forward(speed = 70)
    time.sleep(0.5)
    readings = read_sensors(sensors)
    while(readings[1] == 0 or readings[2] == 0):
        readings = read_sensors(sensors)
        time.sleep(0.01)
    time.sleep(0.05)
    motors[1].off()
    return
    

def turnRight(sensors, motors):
    print("turning right")
    motors[0].off()
    motors[1].off()
    motors[0].Forward(speed = 70)
    time.sleep(0.5)
    readings = read_sensors(sensors)
    while(readings[1] == 0 or readings[2] == 0):
        readings = read_sensors(sensors)
        time.sleep(0.01)
    motors[0].off()
    return

def turn180():
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

def maintainPath(readings, motors):
    if(readings[1] == False):
        motors[0].off()
        motors[0].Forward(speed = 100)
        #print("motorL speed increased")
    else:
        motors[1].off()
        motors[1].Forward(speed = 100)
        #print("motorR speed increased")

def checkForBox(isLeft):
    return

def grabBox():
    return boxColour

def unloadBox():
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
    if(pathTaken[-1] == "rc"):
        newPath.append("l")
    elif(pathTaken[-1] == "lc"):
        newPath.append("r")
    else:
        print("Not on a check branch")
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
    return newPath

def returnPathCalculation(Baypath):
    returnPath = reversePath(Baypath[1:])

    return returnPath
        
def turnDecision(branchProfile, sensors, motors, path, pathIndex):
    global end
    global decisionSkipTime
    global robotStatus
    straightTime = 8
    confirmationTime = 0.05
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
            time.sleep(1)
            end = True
            return pathIndex
        else:
            return pathIndex

    elif branchProfile == [True, True, True]:
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
        if updatedBranchProfile == [True, True, False]:
            print("Left branch")
            print(pathIndex, path[pathIndex])
            if path[pathIndex] == "l":
                turnLeft(sensors, motors)
            elif path[pathIndex] == "s":
                print("Going straight")
                decisionSkipTime = straightTime
            elif path[pathIndex] == "lc":
                if(checkForBox(left = True)):
                    turnLeft()
                    robotStatus = "pickupBox"
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
                if(checkForBox(left = False)):
                    turnRight()
                    robotStatus = "pickupBox"
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

def mainLoop(sensors, button, motors):
    global decisionSkipTime
    global programOn
    global circuitPathIndex
    try:
        while True:
            if(programOn):
                readings = read_sensors(sensors)
                motors[0].off()
                motors[1].off()
                if(robotStatus == "followCircuit"):
                    if(end == False):
                        motors[0].Forward(speed = 60)
                        motors[1].Forward(speed = 60)          
                        if(readings[1] != readings[2]):
                            maintainPath(readings, motors)
                        branchProfile = turnDetector(readings)
                        #print(branchProfile)
                        if(decisionSkipTime == 0):
                            circuitPathIndex = turnDecision(branchProfile, sensors, motors, circuitPath, circuitPathIndex)
                        elif(decisionSkipTime > 0):
                            print(decisionSkipTime)
                            decisionSkipTime -= 1
                        else:
                            decisionSkipTime = 0
                        #print("on path")
                        #print(readings, turnDetector(readings))
                            #print(listIndex)
                            
                        time.sleep(0.02)

                elif(robotStatus == "pickupBox"):
                    if(frontDistance > maxDistance):
                        print("No box detected")
                    elif(frontDistance > distanceToGrab):
                        motors[0].Forward(speed = 30)
                        motors[1].Forward(speed = 30)          
                        if(readings[1] != readings[2]):
                            maintainPath(readings, motors)
                    else:
                        currentBoxColour = grabBox()
                        turn180()
                        robotStatus = "goToBay"
                        bayPath = bayPathCalculation(currentBoxColour, circuitPath, circuitPathIndex)
                    time.sleep(0.02)

                elif(robotStatus == "goToBay"):
                    motors[0].Forward(speed = 60)
                    motors[1].Forward(speed = 60)          
                    if(readings[1] != readings[2]):
                        maintainPath(readings, motors)
                    branchProfile = turnDetector(readings)
                    #print(branchProfile)
                    if(decisionSkipTime == 0):
                        bayPathIndex = turnDecision(branchProfile, sensors, motors, bayPath, bayPathIndex)
                        if(bayPathIndex == len(bayPath)):
                            unloadBox()
                            robotStatus = "returnToCircuit"
                            returnPath = returnPathCalculation(bayPath)
                    elif(decisionSkipTime > 0):
                        print(decisionSkipTime)
                        decisionSkipTime -= 1
                    else:
                        decisionSkipTime = 0
                    time.sleep(0.02)

                elif(robotStatus == "returnToCircuit"):
                    motors[0].Forward(speed = 60)
                    motors[1].Forward(speed = 60)          
                    if(readings[1] != readings[2]):
                        maintainPath(readings, motors)
                    branchProfile = turnDetector(readings)
                    #print(branchProfile)
                    if(decisionSkipTime == 0):
                        returnPathIndex = turnDecision(branchProfile, sensors, motors, returnPath, returnPathIndex)
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
                    programOn = True
    except KeyboardInterrupt:
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
    pin1 = Pin(21, Pin.OUT)
    pin1.value(0)

    pin3 = Pin(19, Pin.OUT)
    pin3.value(0)

    pin4 = Pin(17, Pin.OUT)
    pin4.value(0)

    pin5 = Pin(16, Pin.OUT)
    pin5.value(0)

    pin6 = Pin(14, Pin.OUT)
    pin6.value(0)

    pin7 = Pin(13, Pin.OUT)
    pin7.value(0)

    pin8 = Pin(12, Pin.OUT)
    pin8.value(0)

    pin9 = Pin(11, Pin.OUT)
    pin9.value(0)

    pin10 = Pin(10, Pin.OUT)
    pin10.value(0)

    pin11 = Pin(9, Pin.OUT)
    pin11.value(0)

    pin12 = Pin(8, Pin.OUT)
    pin12.value(0)

    mainLoop(sensors, button, motors)


    







