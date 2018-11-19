
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



# Put the color sensor into COL-REFLECT mode
# to measure reflected light intensity.
# In this mode the sensor will return a value between 0 and 100
sColor1.mode = 'COL-REFLECT'
sColor2.mode = 'COL-REFLECT'

threshold = 10

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

    baseSpeed = 30
    print(PID)

    #
    while(color1 > 10 and color2 > 10):
        if(error < desired):
            mA.duty_cycle_sp = baseSpeed - abs(int(baseSpeed * PID))
        if(error > desired):
            mB.duty_cycle_sp = baseSpeed - abs(int(baseSpeed * PID))



def setMotorSpeed(dutyA, dutyB):
    if(dutyA < 0):
        mA.polarity = mA.POLARITY_INVERSED
        mA.duty_cycle_sp = abs(dutyA)
    else:
        mA.polarity = mA.POLARITY_NORMAL
        mA.duty_cycle_sp = dutyA

    if(dutyB < 0):
        mB.polarity = mB.POLARITY_INVERSED
        mB.duty_cycle_sp = abs(dutyB)
    else:
        mB.polarity = mB.POLARITY_NORMAL
        mB.duty_cycle_sp = dutyB



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

    mA.run_to_rel_pos(position_sp=nA, speed_sp=speed, stop_action="brake")
    mB.run_to_rel_pos(position_sp=nB, speed_sp=speed, stop_action="brake")
    encoderThreshold = 3

    while nA != mA.position and nB != mB.position:
        if(nA < mA.position + encoderThreshold and nA > mA.position + encoderThreshold):
            if(nB < mB.position+encoderThreshold and nB > mB.position + encoderThreshold):
                break;
        posA = mA.position #nnothing

    mA.polarity = mA.POLARITY_NORMAL
    mB.polarity = mB.POLARITY_NORMAL
    mA.run_direct()
    mB.run_direct()


def turn(angle):
    encoderPos = round((angle /360) * mA.count_per_rot) #gets the number of tacho counts to rotate "angle" degrees
    if(angle < 0 ):
        setMotorSpeed(-30,30)

    if(angle > 0):
        setMotorSpeed(30,-30)




def exitCommand():
    setMotorSpeed(0,0)

atexit.register(exitCommand)

#printSample(500)

while True:
    color1 = sColor1.value()
    color2 = sColor2.value()
    sensorDif = getSensorDif(color1,color2)

    #print("Color dif: ", getSensorDif(color1,color2))
    #print("SP pos: ",mA.position_sp)
    #print("pos: ", mA.position)

    #Next state logic
    actionQ = ['u', 'p', 'r', 'l', 'd']

    if(color1 < threshold and color2 < threshold):  #If intersection, go to next state
        nxState = actionQ.pop(0)

    #State logic
    if(nxState == 'u'):
        if abs(getSensorDif(color1, color2)) > threshold:
            lineFollowing(color1, color2, sensorDif)
    if(nxState == 'p'):
        turn(180)
    else:
        setMotorSpeed(50,50)






