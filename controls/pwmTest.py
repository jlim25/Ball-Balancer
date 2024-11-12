import RPi.GPIO as GPIO
from time import sleep
import gpiod

PWM_PIN = 12
chip = gpiod.Chip('gpiochip4')
pwm_line = chip.get_line(PWM_PIN)
pwm_line.request(consumer="motor", type=gpiod.LINE_REQ_DIR_OUT)

pwm_line.set_value(1)
pwm_line.



while 1:
	pwm_line.set_value(1)
	print("on")
	sleep(2)
	pwm_line.set_value(0)
	print("off")
	sleep(2)
	
	
