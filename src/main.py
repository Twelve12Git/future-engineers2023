import sensor, time, pyb
from pyb import Pin, Timer, Servo, LED

def constrain(value, min_, max_):
	if min_ <= value and value <= max_: return value
	return min_ if value < min_ else max_

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

	def __init__(self, outA1, outA2):
		self.outA1 = outA1
		self.outA2 = outA2

	def set_motor(self, speed):  # -100 <= speed <= 100
		if speed > 0:
			self.outA1.pulse_width_percent(100)
			self.outA2.pulse_width_percent(100-speed)
		else:
			speed = abs(speed)
			self.outA1.pulse_width_percent(100-speed)
			self.outA2.pulse_width_percent(100)

class PID:
	_err_sum = 0
	_prev_err = 0
	u = 0
	def __init__(self, p, i, d, min_es = -100, max_es = 100):
		self._p = p
		self._i = i
		self._d = d
		self._min_err_sum = min_es
		self._max_err_sum = max_es

	def __call__(self, err):
		self._err_sum += constrain(err * self._i, self._min_err_sum, self._max_err_sum)
		self.u = self._p*err + self._err_sum + self._d*(err-self._prev_err)
		self._prev_err = err
		return self.u

	def reset():
		self.u = 0
		self._prev_err = 0
		self._err_sum = 0

def atr(roi): # area to roi
	field = ROI_FIELD
	return (field[0]+roi[0], field[1]+roi[1], roi[2], roi[3])

def deb_blobs(img, prnt=True, **blobs):

	for key, value in blobs.items():
		if value and len(value) and None not in value:
			if prnt: print(f"{key}: {*[(x.cx(), x.pixels()) for x in value]}; ", end="\t")
			[img.draw_rectangle(x.rect(), (255, 255, 255)) for x in value]
	if prnt: print("")

def deb(img, **kwargs):
	if kwargs:
		for key, value in kwargs.items():
			print(f"{key}: {value};  ", end="")
		print("")

def deb_roi():
	#img.draw_rectangle(ROI_FIELD, (255, 0, 0)) # поле
	img.draw_rectangle(atr(AREA_WALL_FRONT), (255, 255, 0)) # зона поиска передней стены
	img.draw_rectangle(atr(AREA_WALL_LEFT), (255, 200, 0)) # зона поиска левой стены
	img.draw_rectangle(atr(AREA_WALL_RIGHT), (200, 255, 0)) # зона поиска правой стены
	img.draw_rectangle(atr(AREA_CUBES), (0, 0, 255)) # зона поиска кубиков
	img.draw_rectangle(atr(AREA_TURNS), (255, 0, 255))
	#img.draw_rectangle(atr(AREA_RED_CUBES), (255, 0, 0)) # зона поиска кубиков
	#img.draw_rectangle(atr(AREA_GREEN_CUBES), (0, 255, 0)) # зона поиска кубиков

pwm_timer = Timer(2, freq=1000)
driver = Driver(
	pwm_timer.channel(4, Timer.PWM, pin=Pin("P5")),
	pwm_timer.channel(3, Timer.PWM, pin=Pin("P4"))
)
servo = Servo(2)

main_pid = PID(0.3, 0, 0)


clock = time.clock()

ledr = LED(1)
ledg = LED(2)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQQVGA) # 80 : 60
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, 6000)

GLOBAL_WIDTH = 80
GLOBAL_HIGHT = 60

WIDTH = 80
HIGHT = 40

ROI_FIELD = (0, 25, GLOBAL_WIDTH-0, GLOBAL_HIGHT-25) # x, y, dx, dy
AREA_WALL_FRONT = (35 ,0,WIDTH-35*2 , HIGHT)
AREA_WALL_LEFT = (0, 0, 10, HIGHT)
AREA_WALL_RIGHT = (WIDTH-10, 0, 10, HIGHT)
AREA_CUBES = (0, 0, WIDTH, int(HIGHT*0.35))
AREA_TURNS = (20, int(HIGHT*0.6), WIDTH-20*2, HIGHT-int(HIGHT*0.6))
AREA_RED_CUBES = (0, 0, int(WIDTH*0.8), int(HIGHT*0.35))
AREA_GREEN_CUBES = (WIDTH-int(WIDTH*0.8), 0, int(WIDTH*0.8), int(HIGHT*0.35))

RED = (0, 76, 17, 127, -37, 127)
GREEN = (0, 100, -128, -25, -128, 127)
BLACK = (0, 46, -128, 127, -128, 19)
ORANGE = (0, 100, -128, 127, 13, 95)
BLUE = (0, 100, -128, 127, -67, -17)

left_offset = 130
right_offset = 170
prev_cur_cube = None

cur_millis = 0
force_go_timer = 0
orange_turn_deadtime = blue_turn_deadtime = 0
blue_turns = orange_turns = turns = 0
clockwise = True ## usingg left area by default

button = Pin("P6", Pin.IN)

while button.value():
	img = sensor.snapshot()
	driver.set_motor(0)
	servo.angle(0)

pyb.delay(800)

cur_millis = pyb.millis()
finish_timer = cur_millis + 3500
############ SPEED ############
driver.set_motor(40)
###############################

while cur_millis < finish_timer:
	clock.tick()
	img = sensor.snapshot().lens_corr(strength = 1.8, zoom = 1.0)

	cur_millis = pyb.millis()
	# ищем блобы
	walls_left = img.find_blobs([BLACK], roi=atr(AREA_WALL_LEFT), pixels_threshold=30)
	walls_right = img.find_blobs([BLACK], roi=atr(AREA_WALL_RIGHT), pixels_threshold=30)
	walls_front = img.find_blobs([BLACK], roi=atr(AREA_WALL_FRONT), pixels_threshold=50)
	turn_orange = img.find_blobs([ORANGE], roi=atr(AREA_TURNS), pixels_threshold=20)
	turn_blue = img.find_blobs([BLUE], roi=atr(AREA_TURNS), pixels_threshold=20)
	# считаем полщади, проверяем что не 0
	left_area  = walls_left[0].pixels()  if len(walls_left)  else 0
	right_area = walls_right[0].pixels() if len(walls_right) else 0
	front_area = walls_front[0].pixels() if len(walls_front) else 0

	if len(turn_orange) and orange_turn_deadtime < cur_millis:
		orange_turns += 1
		orange_turn_deadtime = cur_millis + 3500
	if len(turn_blue) and blue_turn_deadtime < cur_millis:
		blue_turns += 1
		blue_turn_deadtime = cur_millis + 3500
	if orange_turns < blue_turns: clockwise = False
	if orange_turns == blue_turns: turns = orange_turns

	if clockwise:
		err = left_offset - (left_area + int(front_area*0.7))
	else:
		err = right_area + int(front_area*0.5) - right_offset

	u = main_pid(err)
	servo.angle(constrain(int(u), -45, 45))

	if turns < 12:
		finish_timer = cur_millis + 2500
	else:
		pass


	deb_roi()
	deb(img, la=left_area, ra=right_area, fa=front_area, err=err, clockwise=clockwise, turns=turns, timer=cur_millis-finish_timer)
	#deb(img, blue=blue_turns, orng=orange_turns, al=turns)
	deb_blobs(img,False, bl=turn_blue, orn=turn_orange, lw=walls_left, rw=walls_right, fw=walls_front)

