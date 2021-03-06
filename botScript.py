
#!/usr/bin/python3
import ev3dev.ev3 as ev3
from time import sleep
import atexit
import time

#position_sp, to read from the position encoders (rate_sp sets the accelleration)
#http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-jessie/motors.html#id21
mA = ev3.LargeMotor('outA')
mB = ev3.LargeMotor('outB')
mA.run_direct()
mB.run_direct()

# Connect EV3 color sensor
#http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-jessie/sensor_data.html#lego-ev3-color
sColor1 = ev3.ColorSensor('in1')
sColor2 = ev3.ColorSensor('in4')

sTouch = ev3.TouchSensor('in2')



# Maps:
eightPushCanMap = "fDrllDlrrDlrrDlrrDlrrDrllDrllDrll"

solutionMap = "fDlrffrfffrfDrllfffDbffrfflfllDlrrfDrllffDrllffDrllDlffrfffrfflffllffDlrrfDrllffDrllDlfffrflfflfllDlrrffDlrrfDrllffDrlflffDbrDlDlrrD"

# Put the color sensor into COL-REFLECT mode
# to measure reflected light intensity.
# In this mode the sensor will return a value between 0 and 100
sColor1.mode = 'COL-REFLECT'
sColor2.mode = 'COL-REFLECT'

threshold = 20

mA.position = 0
mB.position = 0


# Variables for time measurements
tick = time.time()


#Max kp val 0.054
Kp = 0.01
Ki = 0.001
Kd = 0.0001

I_value = 0
previousError = 0


def lineFollowing(color1, color2, sensorDif):
    global tick, I_value, previousError

    desired = 5
    error = desired - sensorDif
    de = error - previousError
    dt = time.time() - tick
    tick = time.time()

    P_value = Kp * error
    #I_value += Ki * (error * dt)
    D_value = Kd * (de / dt)
    PID = P_value + D_value
    #+ I_value + D_value

    baseSpeed = 100
    #print(PID)

    if(error < desired):
        mA.duty_cycle_sp = baseSpeed - abs(int(baseSpeed * PID)) # We can't add and subtract at the same time due to turns
    if(error > desired):
        mB.duty_cycle_sp = baseSpeed - abs(int(baseSpeed * PID))



def setMotorSpeed(dutyA, dutyB):
    if(dutyA < 0):
        mA.polarity = mA.POLARITY_INVERSED
        mA.duty_cycle_sp = abs(dutyA)
    else:
        mA.polarity = mA.POLARITY_NORMAL
        mA.duty_cycle_sp = abs(dutyA)

    if(dutyB < 0):
        mB.polarity = mB.POLARITY_INVERSED
        mB.duty_cycle_sp = abs(dutyB)
    else:
        mB.polarity = mB.POLARITY_NORMAL
        mB.duty_cycle_sp = abs(dutyB)



def getSensorDif(A,B):
    return A-B #sensor value A-B

def printSample(n):
    f = open("WhiteLowLight.txt", "w+")

    for i in range(500):
        color1 = sColor1.value()
        color2 = sColor2.value()
        f.write("%d\n" % color1)
        f.write("%d\n" % color2)

    print("Color1: ", color1)
    print("color2: ", color2)

    f.close()

def printEncoderSample(n):
    f = open("EncoderTest.txt","w+")
    for i in range(n):
        posA = mA.position
        posB = mB.position
        f.write("%d\n" % posA)
        f.write("%d\n" % posB)
    f.close()

def goToEncoderPos(nA, nB, speed, inversed):
    if(inversed == 1):
        mA.polarity = mA.POLARITY_INVERSED
        mB.polarity = mB.POLARITY_INVERSED

    mA.run_to_rel_pos(position_sp=nA+15, speed_sp=speed, stop_action="brake")#10 is a tuning parameter to prevent not
    mB.run_to_rel_pos(position_sp=nB+15, speed_sp=speed, stop_action="brake")#reaching the encoder position
    encoderThreshold = 5

    while True:
        if(nA < mA.position + encoderThreshold and nA < mA.position - encoderThreshold):
            if(nB < mB.position + encoderThreshold and nB < mB.position - encoderThreshold):
                mA.run_direct()
                mB.run_direct()
                return



def isOnLine(sensorData):
    if(sensorData < 10):
        return True
    else:
        return False


def turn(angle):
        mA.position = 0
        mB.position = 0
        goToEncoderPos(mA.position + 40, mB.position + 40, 200,0)

        mA.position = 0
        mB.position = 0
        encoderPosA = mA.position + ((angle/180) * mA.count_per_rot) #gets the number of tacho counts to rotate "angle" degrees
        encoderPosB = mB.position - ((angle / 180) * mB.count_per_rot)
        goToEncoderPos(encoderPosA,encoderPosB,200,0)


def deliverCan():
    # Line-follow until a certain position to accurately position can.
    mA.position = 0
    mB.position = 0
    desiredCanLocation = 440

    while(mA.position < desiredCanLocation):
        color1 = sColor1.value()
        color2 = sColor2.value()
        sensorDif = getSensorDif(color1, color2)
        setMotorSpeed(50,50)
        mA.polarity = mA.POLARITY_NORMAL
        mB.polarity = mA.POLARITY_NORMAL
        if abs(getSensorDif(color1, color2)) > threshold:
            lineFollowing(color1, color2, sensorDif)



    setMotorSpeed(-50, -50)
    while(not(color1 < threshold and color2 < threshold)): # drive backwards
        color1 = sColor1.value()
        color2 = sColor2.value()

    setMotorSpeed(50,50)


def calibration():
    while True:
        color1 = sColor1.value()
        color2 = sColor2.value()
        difference = color1 - color2

        print("Color1: ", color1,  "Color2: ", color2, "Difference: ", difference)


def waitForButtonPush():
    touchValue = 0
    while touchValue != 1:
        touchValue = sTouch.value()

    time.sleep(1)

def exitCommand():
    setMotorSpeed(0,0)

atexit.register(exitCommand)


currentState = 'f'
onLineVar = False
action = 0

#Calibration function
#calibration()

ev3.Sound.beep(500)

#start by pressing the start button
waitForButtonPush()

killSwitch = 0

actionQ = solutionMap

while killSwitch != 1:
    killSwitch = sTouch.value()
    color1 = sColor1.value()
    color2 = sColor2.value()

    sensorDif = getSensorDif(color1,color2)

    #Next state logic

    if(action == len(actionQ)):
        break

    if(color1 < threshold and color2 < threshold and onLineVar == False):  #If intersection, go to next state
        currentState = actionQ[action]
        onLineVar = True
        action = action + 1
    if(color1 > threshold and color2 > threshold and onLineVar == True):
        onLineVar = False

    #State logic
    if(currentState == 'f'):
        mA.polarity = mA.POLARITY_NORMAL
        mB.polarity = mA.POLARITY_NORMAL
        if abs(getSensorDif(color1, color2)) > threshold:
            lineFollowing(color1, color2, sensorDif)
        else:
            setMotorSpeed(70, 70)
    if(currentState == 'r'):
        turn(100)
        currentState = 'f'
    if(currentState == 'l'):
        turn(-95)
        currentState = 'f'
    if(currentState == 'b'):
        turn(210)
        currentState = 'f'
    if(currentState == 'D'):
        deliverCan()
        onLineVar = False
        currentState = 'f'


# Play Victory music:
ev3.Sound.play('/home/robot/SS2.wav')


