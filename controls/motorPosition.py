from gpiozero import PWMOutputDevice
from time import sleep
import gpiod

DEGREES_PER_STEP = 1.8
STEPS_PER_REV = 200
MAX_SPEED = 3 #rev/s
MIN_SPEED = 0.1
MOTOR_ON = 0.5 #duty cycle
MOTOR_OFF = 0
MOTOR_CLOCKWISE = 1
MOTOR_COUNTER_CLOCKWISE = 0
GPIO_ON = 1
GPIO_OFF = 0
PWM_PIN = 12
DIR_PIN = 16

class motorControl:
	def __init__(self, pwm, direction):
		chip = gpiod.Chip('gpiochip4')
		self.dirPin = chip.get_line(direction)
		self.dirPin.request(consumer="dir", type=gpiod.LINE_REQ_DIR_OUT)
		self.dirPin.set_value(GPIO_OFF)
		self.controller = PWMOutputDevice(pwm, frequency=50)
		self.controller.value = MOTOR_OFF #duty cycle 0-1
	
	#speed in Rev/s
	def clampSpeed(speed):
		if(speed>MAX_SPEED):
			speed = MAX_SPEED
			print("speed is too high setting to " + MAX_SPEED)
		elif(speed<MIN_SPEED):
			speed = MIN_SPEED
			print("speed must be > 0")

	#position in degrees and speed in Rev/S
	def setMotorPosition(position, speed, self):
		if(position == 0 or speed == 0):
			#motor should be stopped by setting DC to 0
			self.controller.value = MOTOR_OFF
			return False
		self.clampSpeed(speed)
		frequency = speed*STEPS_PER_REV
		self.controller.frequency = abs(frequency)
		if(position > 0):
			print("dir: clockwise lol")
			self.dirPin.set_value(MOTOR_CLOCKWISE)
		else:
			print("dir: counter-clockwise")
			self.dirPin.set_value(MOTOR_COUNTER_CLOCKWISE)
		self.controller.value = MOTOR_ON
		sleep((abs(position)/360)/speed)


def main():
	motorA = motorControl(PWM_PIN,DIR_PIN)
	while 1:
		position_s = input("Position in degrees: ")
		speed_s = input("Speed in Rev/s: ")
	
		try:
			position = int(position_s)
			speed = float(speed_s)
			print(position)
			print(speed)
			if not motorA.setMotorPosition(position,speed):
				continue
		except ValueError:
				print("input is not an int!")

