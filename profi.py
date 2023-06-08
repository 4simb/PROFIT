import sensor, image, time, pyb
import math
from image import SEARCH_EX, SEARCH_DS
from pyb import Timer
from machine import Pin

maximum = 100
threshold_black = (0, 70)

kpBias = 0.29 #0.23
kdBias = 0.81 #1.5
kpCentral = 0.38 #0.36 #0.41 #0.38 #0.25
kdCentral = 10 #22 #45

errBias = 0
errCentral = 0
errBiasOld = 0
errCentralOld = 0

biasThreshold = 30 #6
centralThreshold = 10

vel = 90 #27
leftU = 0
rightU = 0

uBias = 0
uCentral = 0
uCentralSoft = 0
uBiasSoft = 0

timer = pyb.Timer(4, freq=1000)
left1 = timer.channel(1, pyb.Timer.PWM, pin = pyb.Pin("P7"))
left2 = timer.channel(2, pyb.Timer.PWM, pin = pyb.Pin("P8"))

timer2 = pyb.Timer(2, freq=1000)
right1 = timer2.channel(3, pyb.Timer.PWM, pin = pyb.Pin("P4"))
right2 = timer2.channel(4, pyb.Timer.PWM, pin = pyb.Pin("P5"))

tim = pyb.Timer(4, freq=50, period = 4000)
impeller = tim.channel(3, pyb.Timer.PWM, pin = pyb.Pin("P9"))
impeller.pulse_width(2000)

def setMotors(p1, p2, value):
    if value >= maximum:
        p1.pulse_width_percent(maximum)
        p2.pulse_width_percent(0)
    elif value >= 0:
        p1.pulse_width_percent(int(value))
        p2.pulse_width_percent(0)
    elif value < 0 and value > -maximum:
        p1.pulse_width_percent(0)
        p2.pulse_width_percent(abs(int(value)))
    else:
        p1.pulse_width_percent(0)
        p2.pulse_width_percent(maximum)

sensor.reset()
sensor.set_contrast(1)                     # Reset and initialize the sensor.
sensor.set_gainceiling(16)
sensor.set_pixformat(sensor.GRAYSCALE) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

for i in range(2400, 2672):
    impeller.pulse_width(i)
    pyb.delay(5)
#impeller.pulse_width_percent(100)

while(True):
    clock.tick()                    # Update the FPS clock.
    img = sensor.snapshot()

    roiUp = (0, 8, img.width(), int(0.5 * img.height()))
    roiDown = (0, int(0.5 * img.height()), img.width(), int(0.5 * img.height()))

    maxiUp = 0
    upBlob = []
    for yb in img.find_blobs([threshold_black], merge = True, margin = 60, roi = roiUp):
        if yb.pixels() > maxiUp:
            maxiUp = yb.pixels()
            upBlob = yb

    maxiDown = 0
    downBlob = []
    for yb in img.find_blobs([threshold_black], merge = True, margin = 60, roi = roiDown):
        if yb.pixels() > maxiDown:
            maxiDown = yb.pixels()
            downBlob = yb

    if downBlob != []:
        img.draw_rectangle((downBlob.x(), downBlob.y(), downBlob.w(), downBlob.h()))
    if upBlob != []:
        img.draw_rectangle((upBlob.x(), upBlob.y(), upBlob.w(), upBlob.h()))

    if downBlob != [] and upBlob != []:
        img.draw_line(upBlob.cx(), upBlob.cy(), downBlob.cx(), downBlob.cy())
        errBias = upBlob.cx() - downBlob.cx();
        p = errBias * kpBias
        d = (errBias - errBiasOld) * kdBias
        uBias = p + d

        if abs(uBias) > biasThreshold:
            if uBias < 0:
                uBias = -biasThreshold
            else:
                uBias = biasThreshold

        #0.06 0.94
        uBiasSoft = uBiasSoft * 0.5 + uBias * 0.5
        errBiasOld = errBias

    if downBlob != []:
        errCentral = downBlob.x() - 0.5 * img.width()
    elif downBlob == [] and upBlob != []:
        errCentral = upBlob.x() - 0.5 * img.width()

    pCentral = errCentral * kpCentral
    dCentral = (errCentral - errCentralOld) * kdCentral
    uCentral = pCentral + dCentral
    errCentralOld = errCentral

    if abs(uCentral) > centralThreshold:
        if uCentral < 0:
            uCentral = -centralThreshold
        else:
            uCentral = centralThreshold

    #0.4 0.6
    uCentralSoft = 0.4 * uCentralSoft + 0.6 * uCentral
    #0.05 0.95
    u = 0.11 * uBiasSoft + 0.89 * uCentralSoft
    #u *= vel/7 #6.5

    v = vel - pow(abs(uCentral), 1.8) * 0.001 #0.085 #0.4 #0.82
    if abs(errCentral) < 47: #33
        v = maximum

    u *= v/7.82 #7.3
    leftU = v + u
    rightU = v - u
    setMotors(left1, left2, leftU)
    setMotors(right1, right2, rightU)

    #print(clock.fps())
    #pyb.delay(1)
