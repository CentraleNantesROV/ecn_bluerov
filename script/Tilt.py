import sys
import time

import navio2.pwm
import navio2.util
from numpy import sin, pi

navio2.util.check_apm()

PWM_OUTPUT = 12
SERVO_D = 0.4
dt = .01
t = 0
f = 0.1

if len(sys.argv) == 2:
    f = float(sys.argv[1])

with navio2.pwm.PWM(PWM_OUTPUT) as pwm:
    pwm.set_period(50)
    pwm.enable()

    pwm.set_duty_cycle(1.500)
    time.sleep(2)

    if f != 0:
        while (True):
            pwm.set_duty_cycle(1.500 + SERVO_D*sin(t*f*2*pi))
            time.sleep(dt)
            t += dt
    else:
        pwm.disable()
