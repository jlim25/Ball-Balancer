from gpiozero import PWMOutputDevice
from time import sleep
import gpiod

MOTOR_ON = 0.5
MOTOR_OFF = 0
MOTOR_CLOCKWISE = 1
MOTOR_COUNTER_CLOCKWISE = 0
GPIO_ON = 1
GPIO_OFF = 0
PWM_PIN = 12
DIR_PIN = 16

motor = PWMOutputDevice(PWM_PIN, frequency=100)
motor.value = MOTOR_OFF #duty cycle 0-1

chip = gpiod.Chip('gpiochip4')
dir_line = chip.get_line(DIR_PIN)
dir_line.request(consumer="dir", type=gpiod.LINE_REQ_DIR_OUT)
dir_line.set_value(GPIO_OFF)

while 1:
	speed_s = input("Input int for speed :")
	try:
		speed = int(speed_s)
		print("speed: "+speed_s)
		if(speed == 0):
			#motor should be stopped by setting DC to 0
			motor.value = MOTOR_OFF
		elif(speed > 0):
			print("dir: clockwise")
			motor.value = MOTOR_ON
			dir_line.set_value(MOTOR_CLOCKWISE)
			motor.frequency = abs(speed)
		else:
			print("dir: counter-clockwise")
			motor.value = MOTOR_ON
			dir_line.set_value(MOTOR_COUNTER_CLOCKWISE)
			motor.frequency = abs(speed)
	except ValueError:
		print("input is not an int!")
	sleep(1)
