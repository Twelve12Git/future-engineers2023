from pyb import Pin, Timer, Servo

class Driver:
    """
	get PWM timer IN out ports (ex Timer(4, freq=1000).channel(3, Timer.PWM, pin=Pin("P9")))
	OpenMV Cam M4 allowed PWM timers: TIM2, TIM3, TIM4
	Timer Pinout:
	Timer 1 Channel 3 Negative -> P0 (PB15)
	Timer 1 Channel 2 Negative -> P1 (PB14)
	Timer 1 Channel 1 Negative -> P2 (PB13)
	Timer 2 Channel 3 Positive -> P4 (PB10)
	Timer 2 Channel 4 Positive -> P5 (PB11)
	Timer 2 Channel 1 Positive -> P6 (PA5)
	Timer 4 Channel 1 Negative -> P7 (PD12)
	Timer 4 Channel 2 Negative -> P8 (PD13)
	Timer 4 Channel 3 Positive -> P9 (PD14) (OpenMV Cam M7/H7 Only - Not OpenMV Cam H7 Plus)
	"""
	def __init__(self, outA1, outA2, fast_decay=True):
	    self.outA1 = outA1
		self.outA2 = outA2
		self.fast_decay = fast_decay

    def set_motor(self, speed): # -100 <= speed <= 100
	    if self.fast_decay:
		    if speed > 0:
			    self.outA1.pulse_width_percent(speed)
				self.outA2.pulse_width_percent(0)
			else:
			    speed = abs(speed)
				self.outA1.pulse_width_percent(0)
				self.outA2.pulse_width_percent(speed)
		else:
		    if speed > 0:
			    self.outA1.pulse_width_percent(1)
				self.outA2.pulse_width_percent(speed)
			else:
			    speed = abs(speed)
				self.outA1.pulse_width_percent(speed)
				self.outA2.pulse_width_percent(1)


pwm_timer = Timer(4, freq=1000)
motor_port1 = pwm_timer.channel(1, Timer.PWM, pin=Pin("P7"))
motor_port2 = pwm_timer.channel(3, Timer.PWM, pin=Pin("P9"))
driver = Driver(motor_port1, motor_port2)
servo = Servo(2) # P8


def main():
    servo.angle(0)
	driver.set_motor(50)

while(True):
    main()
