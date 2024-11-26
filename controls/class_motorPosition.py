from gpiozero import PWMOutputDevice
from time import sleep
import threading
import gpiod
import sys

MICROSTEPS = 8 #defined by MC MS1 and MS2 pins
DEGREES_PER_STEP = 1.8/MICROSTEPS
STEPS_PER_REV = 200*MICROSTEPS #motor has 200 steps/rev but MC allows for microstepping
MAX_SPEED = 1.5 #rev/s, determined experimentally
MIN_SPEED = 0.1
MOTOR_ON = 0.5 #duty cycle
MOTOR_OFF = 0
MOTOR_CLOCKWISE = 1
MOTOR_COUNTER_CLOCKWISE = 0
GPIO_ON = 1
GPIO_OFF = 0
MOTOR_A_PWM_PIN = 12
MOTOR_A_DIR_PIN = 6
MOTOR_B_PWM_PIN = 1
MOTOR_B_DIR_PIN = 0
MOTOR_C_PWM_PIN = 8
MOTOR_C_DIR_PIN = 11

class motorControl:
	def __init__(self, pwm, direction):
		chip = gpiod.Chip('gpiochip4')
		self.dirPin = chip.get_line(direction)
		self.dirPin.request(consumer="dir", type=gpiod.LINE_REQ_DIR_OUT)
		self.dirPin.set_value(GPIO_OFF)
		self.controller = PWMOutputDevice(pwm, frequency=50)
		self.controller.value = MOTOR_OFF #duty cycle 0-1

	#speed in Rev/s
	def clampSpeed(self, speed):
		if(speed > MAX_SPEED):
			speed = MAX_SPEED
			print("speed is too high setting to " + MAX_SPEED)
		elif(speed < MIN_SPEED):
			speed = MIN_SPEED
			print("speed must be > 0")

	#position in degrees and speed in Rev/S
	def setMotorPosition(self, position, speed):
		if(position == 0 or speed == 0):
			#motor should be stopped by setting DC to 0
			self.controller.value = MOTOR_OFF
			print("stopping motor")
			return False
		self.clampSpeed(speed)
		frequency = speed*STEPS_PER_REV
		self.controller.frequency = abs(frequency)
		if(position > 0):
			print("dir: clockwise")
			self.dirPin.set_value(MOTOR_CLOCKWISE)
		else:
			print("dir: counter-clockwise")
			self.dirPin.set_value(MOTOR_COUNTER_CLOCKWISE)
		self.controller.value = MOTOR_ON
		sleep((abs(position)/360)/speed)
		self.controller.value = MOTOR_OFF
	
	#speed in Rev/s
	def setMotorSpeed(self, speed):
		if(speed == 0):
			self.controller.value = MOTOR_OFF
			print("stopping motor")
			return None

		self.clampSpeed(abs(speed))
		frequency = abs(speed*STEPS_PER_REV)
		self.controller.frequency = frequency

		if(speed > 0):
			print("dir: clockwise")
			self.dirPin.set_value(MOTOR_CLOCKWISE)
		else:
			print("dir: counter-clockwise")
			self.dirPin.set_value(MOTOR_COUNTER_CLOCKWISE)
		self.controller.value = MOTOR_ON



def moveMotorA():
	motorA = motorControl(MOTOR_A_PWM_PIN,MOTOR_A_DIR_PIN)
	motorDir = -1
	while(1):
		motorA.setMotorPosition(motorDir*15,0.1)
		motorDir*=-1
		if motorDir < 0:
			print("spinning motorA CCW")
		else:
			print("spinning motorA CW")

def moveMotorB():
	motorB = motorControl(MOTOR_B_PWM_PIN,MOTOR_B_DIR_PIN)
	motorDir = -1
	while(1):
		motorB.setMotorPosition(motorDir*15,0.1)
		motorDir*=-1
		if motorDir < 0:
			print("spinning motorB CCW")
		else:
			print("spinning motorB CW")

def moveMotorC():
	motorC = motorControl(MOTOR_C_PWM_PIN,MOTOR_C_DIR_PIN)
	motorDir = -1
	while(1):
		motorC.setMotorPosition(motorDir*15,0.1)
		motorDir*=-1
		if motorDir < 0:
			print("spinning motorC CCW")
		else:
			print("spinning motorC CW")


def main():
	motorA = motorControl(MOTOR_A_PWM_PIN,MOTOR_A_DIR_PIN)
	motorB = motorControl(MOTOR_B_PWM_PIN,MOTOR_B_DIR_PIN)
	motorC = motorControl(MOTOR_C_PWM_PIN,MOTOR_C_DIR_PIN)
	while 1:
		try:
			motorSelection = input("Select motor (A,B,C):")
			position_s = input("Position in degrees: ")
			speed_s = input("Speed in Rev/s: ")
	
			try:
				position = int(position_s)
				speed = float(speed_s)
				print(position)
				print(speed)
				match motorSelection:
					case "A":
						if not motorA.setMotorPosition(position,speed):
							continue
					case "B":
						if not motorB.setMotorPosition(position,speed):
							continue
					case "C":
						if not motorC.setMotorPosition(position,speed):
							continue
					case _:
						print("motor selection invalid!")
						continue
			except ValueError:
					print("input is not an int!")
		except KeyboardInterrupt:
			motorA.setMotorSpeed(0)
			motorB.setMotorSpeed(0)
			motorC.setMotorSpeed(0)

			# motorA.controller.off()
			# motorB.controller.off()
			# motorC.controller.off()
			print("stopping program")
			sys.exit()

def motorMoveDemo():
	motorAThreadHandle = threading.Thread(target = moveMotorA)
	motorBThreadHandle = threading.Thread(target = moveMotorB)
	motorCThreadHandle = threading.Thread(target = moveMotorC)

	motorAThreadHandle.start()
	motorBThreadHandle.start()
	motorCThreadHandle.start()

	motorAThreadHandle.join()
	
	motorBThreadHandle.join()
	motorCThreadHandle.join()

if __name__=="__main__":
    main()
	#motorMoveDemo()
	

