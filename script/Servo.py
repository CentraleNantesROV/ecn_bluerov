import sys
import time

import navio2.pwm
import navio2.util

navio2.util.check_apm()

PWM_OUTPUT = 0
SERVO_MIN = 1.400 #ms
SERVO_MAX = 1.600 #ms

if len(sys.argv) == 2:
    cand = int(sys.argv[1])
    if cand >=0 and cand < 14:
        PWM_OUTPUT = cand

with navio2.pwm.PWM(PWM_OUTPUT) as pwm:
    pwm.set_period(50)
    pwm.enable()

    pwm.set_duty_cycle(1.500)
    time.sleep(2)

    while (True):
        pwm.set_duty_cycle(SERVO_MIN)
        time.sleep(1)
        pwm.set_duty_cycle(SERVO_MAX)
        time.sleep(1)
