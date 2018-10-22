
#!/usr/bin/python3
import ev3dev.ev3 as ev3
from time import sleep
from state import state


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

threshold = 25



def setMotorSpeed(dutyA, dutyB):
    mA.duty_cycle_sp = dutyA
    mB.duty_cycle_sp = dutyB

while True:
    color1 = sColor1.value()
    color2 = sColor2.value()
    print("Color1: ", color1)
    print("color2: ", color2)

    if color1 >= threshold and color2 >= threshold: #proceed the same direction
        setMotorSpeed(20,20)
    if color1 < threshold and color2 > threshold: #turn right
        setMotorSpeed(30, 10)
    if color1 >= threshold and color2 <= threshold: #turn left
        setMotorSpeed(10, 30)
    if color1 < threshold and color2 < threshold: #reached intersection; STOP
        setMotorSpeed(0, 0)

    sleep(0.01)