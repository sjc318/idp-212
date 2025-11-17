import machine
#from gpiozero import DigitalInputDevice
import time
import random
#from turn_detector.py import turnDetector
from machine import Pin, PWM
from utime import sleep
import sys
import select

decisionList = ["l","s","r","s","s","s","s","s","s","s","s","s","s","s","s","s","r","s","l","s"] # left, right or straight
listIndex = 0
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

def turnLeft(sensor0, sensor1, sensor2, sensor3, motorL, motorR):
    print("turning left")
    motorL.off()
    motorR.off()
    motorR.Forward(speed = 70)
    time.sleep(0.1)
    readings = read_sensors(sensor0, sensor1, sensor2, sensor3)
    while(readings[1] == 0 or readings[2] == 0):
        readings = read_sensors(sensor0, sensor1, sensor2, sensor3)
        time.sleep(0.01)
    motorR.off()
    return
    

def turnRight(sensor0, sensor1, sensor2, sensor3, motorL, motorR):
    print("turning right")
    motorL.off()
    motorR.off()
    motorL.Forward(speed = 70)
    time.sleep(0.1)
    readings = read_sensors(sensor0, sensor1, sensor2, sensor3)
    while(readings[1] == 0 or readings[2] == 0):
        readings = read_sensors(sensor0, sensor1, sensor2, sensor3)
        time.sleep(0.01)
    motorL.off()
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
        motorL.off()
        motorL.Forward(speed = 100)
        #print("motorL speed increased")
    else:
        motorR.off()
        motorR.Forward(speed = 100)
        #print("motorR speed increased")
        
def turnDecision(branchProfile, sensor0, sensor1, sensor2, sensor3, motorL, motorR):
    global listIndex
    global end
    global decisionSkipTime
    straightTime = 8
    confirmationTime = 0.05
    if branchProfile == [False, True, False]:
        return
    elif branchProfile == [False, False, False]:
        return
    elif branchProfile == [True, False, True]:
        print("T")
        print(listIndex, decisionList[listIndex])
        if decisionList[listIndex] == "r":
            turnRight(sensor0, sensor1, sensor2, sensor3, motorL, motorR)
            listIndex += 1
            return
        elif decisionList[listIndex] == "l":
            turnLeft(sensor0, sensor1, sensor2, sensor3, motorL, motorR)
            listIndex += 1
            return
        elif decisionList[listIndex] == "s":
            time.sleep(1)
            end = True
            return
        else:
            return

    elif branchProfile == [True, True, True]:
        decisionSkipTime = straightTime
        return
    time.sleep(confirmationTime)
    readings = read_sensors(sensor0, sensor1, sensor2, sensor3)
    updatedBranchProfile = turnDetector(readings)
    if branchProfile == [True, False, False]:
        if updatedBranchProfile == [True, False, False]:
            turnLeft(sensor0, sensor1, sensor2, sensor3, motorL, motorR)
        return

    elif branchProfile == [False, False, True]:
        if updatedBranchProfile == [False, False, True]:
            turnRight(sensor0, sensor1, sensor2, sensor3, motorL, motorR)
        return

    elif branchProfile == [True, True, False]:
        if updatedBranchProfile == [True, True, False]:
            print("Left branch")
            print(listIndex, decisionList[listIndex])
            if decisionList[listIndex] == "l":
                turnLeft(sensor0, sensor1, sensor2, sensor3, motorL, motorR)
                listIndex += 1
                return
            elif decisionList[listIndex] == "s":
                print("Going straight")
                listIndex += 1
                decisionSkipTime = straightTime
                return
        return

    elif branchProfile == [False, True, True]:
        if updatedBranchProfile == [False, True, True]:
            print("Right branch")
            print(listIndex, decisionList[listIndex])
            if decisionList[listIndex] == "r":
                turnRight(sensor0, sensor1, sensor2, sensor3, motorL, motorR)
                listIndex += 1
                return
            elif decisionList[listIndex] == "s":
                print("Going straight")
                listIndex += 1
                decisionSkipTime = straightTime
                return
        return

    else:
        return

            
        
    

def read_sensors(sensor0, sensor1, sensor2, sensor3):
    # Simulate line detection
    # 0 = line detected (black), 1 = white ground
    sensorFL = read_sensor(sensor0)
    sensorML = read_sensor(sensor1)
    sensorMR = read_sensor(sensor2)
    sensorFR = read_sensor(sensor3)
    return [sensorFL, sensorML, sensorMR, sensorFR]

def key_pressed():
    # Non-blocking key press check
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    return bool(dr)

def mainLoop(sensor0, sensor1, sensor2, sensor3, motorL, motorR):
    global listIndex
    global decisionSkipTime
    global programOn
    
    try:
        while True:
            if(end == False):
                readings = read_sensors(sensor0, sensor1, sensor2, sensor3)
                motors = [motorL, motorR]
                motorL.off()
                motorR.off()
                motorL.Forward(speed = 60)
                motorR.Forward(speed = 60)          
                if(readings[1] != readings[2]):
                    maintainPath(readings, motors)
                branchProfile = turnDetector(readings)
                #print(branchProfile)
                if(decisionSkipTime == 0):
                    turnDecision(branchProfile, sensor0, sensor1, sensor2, sensor3, motorL, motorR)
                elif(decisionSkipTime > 0):
                    print(decisionSkipTime)
                    decisionSkipTime -= 1
                else:
                    decisionSkipTime = 0
                #print("on path")
                #print(readings, turnDetector(readings))
                    #print(listIndex)
                    
                time.sleep(0.02)
    except KeyboardInterrupt:
        motorL.off()
        motorR.off()

if __name__ == "__main__":
    sensor0 = setup_sensor(28) #blue
    sensor1 = setup_sensor(26) #purple
    sensor2 = setup_sensor(22) #gray
    sensor3 = setup_sensor(27) #brown
    button = setup_sensor(20)
    motorL = setup_motor(4, 5)
    motorR = setup_motor(7, 6)
    pin1 = Pin(21, Pin.OUT)
    pin1.value(0)

    pin2 = Pin(20, Pin.OUT)
    pin2.value(0)

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

    mainLoop(sensor0, sensor1, sensor2, sensor3, button, motorL, motorR)


    



