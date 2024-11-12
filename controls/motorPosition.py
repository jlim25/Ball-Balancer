from gpiozero import PWMOutputDevice
from time import sleep
import gpiod

DEGREES_PER_STEP = 1.8
STEPS_PER_REV = 200
MAX_SPEED = 3 #rev/s
MIN_SPEED = 0
MOTOR_ON = 0.5 #duty cycle
MOTOR_OFF = 0
MOTOR_CLOCKWISE = 1
MOTOR_COUNTER_CLOCKWISE = 0
GPIO_ON = 1
GPIO_OFF = 0
PWM_PIN = 12
DIR_PIN = 16

#speed in Rev/s
def clampSpeed(speed):
	if(speed>MAX_SPEED):
		speed = MAX_SPEED
		print("speed is too high setting to 2")
	elif(speed<MIN_SPEED):
		speed = MIN_SPEED
		print("speed must be > 0")

#position in degrees and speed in Rev/S
def setMotorPosition(position, speed):
	frequency = speed*STEPS_PER_REV
	motor.frequency = abs(frequency)
	if(position > 0):
		print("dir: clockwise lol")
		dir_line.set_value(MOTOR_CLOCKWISE)
	else:
		print("dir: counter-clockwise")
		dir_line.set_value(MOTOR_COUNTER_CLOCKWISE)
	motor.value = MOTOR_ON
	sleep((abs(position)/360)/speed)
	
	

motor = PWMOutputDevice(PWM_PIN, frequency=50)
motor.value = MOTOR_OFF #duty cycle 0-1

chip = gpiod.Chip('gpiochip4')
dir_line = chip.get_line(DIR_PIN)
dir_line.request(consumer="dir", type=gpiod.LINE_REQ_DIR_OUT)
dir_line.set_value(GPIO_OFF)

while 1:
	motor.value = MOTOR_OFF
	position_s = input("Position in degrees: ")
	speed_s = input("Speed in Rev/s: ")
	
	try:
		position = int(position_s)
		speed = float(speed_s)
		print(position)
		print(speed)
		
		if(position == 0 or speed == 0):
			#motor should be stopped by setting DC to 0
			motor.value = MOTOR_OFF
			continue
			
		clampSpeed(speed)
		setMotorPosition(position, speed)
	except ValueError:
		print("input is not an int!")
