# ----------------------------------------------------------------------
# MicroDev Project: Bigfoot
# Date: May 2, 2022
# Description: Handles GPIO's
# Authors:
# Nathan Hanchey
# Dylan Vetter
# Connor Inglet
# Corey Noura
#
# References:
# raspberry-projects.com/pi/programming-in-python/i2c-programming-in-python/using-the-i2c-interface-2
# ----------------------------------------------------------------------

import sys
import signal
import time
import RPi.GPIO as GPIO
import board
import busio
import adafruit_ina219

import smbus

bus = smbus.SMBus(1)
GPIO.setwarnings(False)


# ********************************** SET BIGFOOT ADC *****************************************
# ----------------------------------------------------------------------
# Description: SET ADC value
# Accepts: Voltage for DAC Output
# Returns: NA
# ----------------------------------------------------------------------
VOUT = 3.3


def set_vout(vout):
    global VOUT
    VOUT = vout


# ******************************** Set for Subject Board Mux  ************************************
# ----------------------------------------------------------------------
# set_mux_add
# Enable Mux  state = 1
# Disable Mux state = 0
# @parameter state
# @parameter enable
# @parameter add
# ----------------------------------------------------------------------
def set_mux_add(state, enable, add):
	GPIO.setmode(GPIO.BCM)
	
	# Reset mux GPIO's to off state
	GPIO.setup(20, GPIO.OUT)
	GPIO.output(20, GPIO.LOW)
	
	GPIO.setup(16, GPIO.OUT)
	GPIO.output(16, GPIO.LOW)
	
	GPIO.setup(26, GPIO.OUT)
	GPIO.output(26, GPIO.LOW)
	
	GPIO.setup(13, GPIO.OUT)
	GPIO.output(13, GPIO.LOW)	
	
	GPIO.setup(19, GPIO.OUT)
	GPIO.output(19, GPIO.LOW)
	
	GPIO.setup(12, GPIO.OUT)
	GPIO.output(12, GPIO.LOW)
	
	GPIO.setup(5, GPIO.OUT)
	GPIO.output(5, GPIO.LOW)
	
	GPIO.setup(0, GPIO.OUT)
	GPIO.output(0, GPIO.LOW)

	GPIO.setup(1, GPIO.OUT)
	GPIO.output(1, GPIO.LOW)	
	
	GPIO.setup(7, GPIO.OUT)
	GPIO.output(7, GPIO.LOW)
	
	GPIO.setup(8, GPIO.OUT)
	GPIO.output(8, GPIO.LOW)
		
	# Conditional for state [on or off]
	if state==1:
		# convert add to binary
		# conditional for each add bit
		if (add & 1) == 1:
			GPIO.output(16, GPIO.HIGH)
			#print("bit0")
		if (add & 2) == 2:
			GPIO.output(26, GPIO.HIGH)
			#print("bit1")
		if (add & 4) == 4:
			GPIO.output(20, GPIO.HIGH)
			#print("bit2")
	
		# Conditional for enable
		if enable==1:
			GPIO.output(8, GPIO.HIGH)
		elif enable==2:
			GPIO.output(7, GPIO.HIGH)
		elif enable==3:
			GPIO.output(1, GPIO.HIGH)
		elif enable==4:
			GPIO.output(0, GPIO.HIGH)
		elif enable==5:
			GPIO.output(5, GPIO.HIGH)
		elif enable==6:
			GPIO.output(12, GPIO.HIGH)
		elif enable==7:
			GPIO.output(19, GPIO.HIGH)
		elif enable==8:
			GPIO.output(13, GPIO.HIGH)
			
	else:
		print("gpios off")
	

# ----------------------------------------------------------------------
# I2C Communication Config
# Parameters: NA
# Returns: NA
# ----------------------------------------------------------------------
def rpi_i2c_config():
	# GPIO reset
	set_mux_add(0, 1, 7)
	
	# ADC reset
	adc_enable(0)
	adc_load(0)
	bus.write_byte(0x48, 0x1c)
	
	# DAC reset
	dac_enable(0)
	rpi_i2c_dac(0x00)
	
	# Current Set
	GPIO.setwarnings(False)
	low_current(0)
	high_current(0)
	# rpi_i2c_ina219(0)
	# print("i2c config complete")


# ----------------------------------------------------------------------
# ADC Capture
# Parameters: NA
# Returns: adc value (volts)
# ----------------------------------------------------------------------
def rpi_i2c_adc():
	# 0x4D ADDRESS? 0x48
	# Read 1st byte
	# read second byte with read_i2c block data
	read_byte = bus.read_byte(0x4C)
	read_word = bus.read_word_data(0x4C, 0)
	
	# combine first and second byte
	read = ((read_word & 0xFF) << 8) | ((read_word & 0xFF00) >> 8)
	read = read >> 2

	# Convert into Voltage
	print("ADC VAL: " + str((read * 5.2) / 1023.0))
	read = (read * 5.2) / 1023.0
	return read


# ----------------------------------------------------------------------
# DAC Set
# Parameters: DAC value to set (4 bit value)
# Returns: NA
# ----------------------------------------------------------------------
def rpi_i2c_dac():
	# 0x0D ADDRESS
	global VOUT
	Vout = VOUT * 788
	v1 = (int(Vout) & 0x0F00) >> 8
	v2 = (int(Vout) & 0x00FF)
	write = bus.write_word_data(0x0D, v1, v2)
	# print("dac set")


# ----------------------------------------------------------------------
# REF: rototron.info/raspberry-pi-ina219-tutorial/
# INA219 Capture
# Parameters: shunt type ( 0 = high current | 1 = low current )
# Returns: current value in amps
# ----------------------------------------------------------------------
def rpi_i2c_ina219(shunt):
	
	i2c = busio.I2C(board.SCL, board.SDA)
	sensor = adafruit_ina219.INA219(i2c)
	
	# print("Bus Voltage: {} V".format(sensor.bus_voltage))
	print("Shunt Voltage: {} V".format(sensor.shunt_voltage))
	# print("Current: {} mA".format(sensor.current))
	
	# Current = shunt voltage / shunnt resistance
	# Conditional for high or low current shunt
	if shunt == 1:
		# Res. 0.04
		print("high current")
		# Current = (shunt voltage / 0.04) * 1000 mV
		# Account for Inaccurate Voltage Drop Over Resisters=>
		current = (sensor.shunt_voltage) / 0.00004
		current = current - (0.2186 * current - 3.7476)
		
	elif shunt == 0:
		# Res. 40.24
		print("low current")

		# Account for Pi Filter =>
		# current = ((sensor.shunt_voltage + 0.000236) / 0.04024) - 0.028 	# Board 3
		# Without Pi Filter Circuit =>
		current = (sensor.shunt_voltage) / 0.04024
		current = current - (186 * current) - 3.7476  # Board 1 / 3 / 4
	
	# print("ina219 current: " + str(current))
	return current


# ******************************** Peripheral Enables ************************************

# ----------------------------------------------------------------------
# High Current
# GPIO 17
# Enable: state = 1
# Disable: state = 0
# ----------------------------------------------------------------------
def high_current(state):	
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(17, GPIO.OUT)
	
	if state==1:
		GPIO.output(17, GPIO.HIGH)
	else:
		GPIO.output(17, GPIO.LOW)
	# print("high current test")


# ----------------------------------------------------------------------
# Low Current
# GPIO 27
# Enable: state = 1
# Disable: state = 0
# ----------------------------------------------------------------------
def low_current(state):	
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(27, GPIO.OUT)
	GPIO.output(27, GPIO.LOW)
	
	if state==1:
		GPIO.output(27, GPIO.HIGH)
	else:
		GPIO.output(27, GPIO.LOW)


# ----------------------------------------------------------------------
# DAC Enable
# GPIO 22
# Enable: state = 1
# Disable: state = 0
# ----------------------------------------------------------------------
def dac_enable(state):
	
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(22, GPIO.OUT)
	
	if state==1:
		GPIO.output(22, GPIO.HIGH)
	else:
		GPIO.output(22, GPIO.LOW)


# ----------------------------------------------------------------------
# ADC with LOAD
# GPIO 10
# Enable: state = 1
# Disable: state = 0
# ----------------------------------------------------------------------
def adc_load(state):
	
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(10, GPIO.OUT)
	
	if state==1:
		GPIO.output(10, GPIO.HIGH)
	else:
		GPIO.output(10, GPIO.LOW)


# ----------------------------------------------------------------------
# ADC without LOAD
# GPIO 9
# Enable: state = 1
# Disable: state = 0
# ----------------------------------------------------------------------
def adc_enable(state):
	
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(9, GPIO.OUT)
	
	if state==1:
		GPIO.output(9, GPIO.HIGH)
	else:
		GPIO.output(9, GPIO.LOW)


# ******************************** Button Interrupts ************************************
def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)


# Globals for Buttons
B1_GPIO = 14
B2_GPIO = 15
B3_GPIO = 18
button_state = 0


# ----------------------------------------------------------------------
# Button 1 Interrupt Handler
# Returns : Changes button state global
# ----------------------------------------------------------------------
def b1_release(channel):
	# Extra Comment
	global button_state, B1_GPIO
	button_state |= 1
	GPIO.remove_event_detect(B1_GPIO)
	print("B1 Pressed")


# ----------------------------------------------------------------------
# Button 2 Interrupt Handler
# Returns : Changes button state global
# ----------------------------------------------------------------------
def b2_release(channel):
	global button_state, B2_GPIO
	button_state |= 2
	GPIO.remove_event_detect(B2_GPIO)
	print("B2 Pressed")


# ----------------------------------------------------------------------
# Button 3 Interrupt Handler
# Returns : Changes button state global
# ----------------------------------------------------------------------
def b3_release(channel):
	global button_state, B3_GPIO
	button_state |= 4
	GPIO.remove_event_detect(B3_GPIO)
	print("B3 Pressed")


# ----------------------------------------------------------------------
# Button 1 Interrupt Enable
# Returns : Changes button state global
# ----------------------------------------------------------------------
def b1_enable():
	global B1_GPIO, button_state
	GPIO.setup(B1_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	try:
		GPIO.add_event_detect(B1_GPIO, GPIO.RISING, callback=b1_release, bouncetime=200)
	except Exception:
			pass
	finally:
		signal.signal(signal.SIGINT, signal_handler)
		button_state &= ~1
		print("b1 enable")


# ----------------------------------------------------------------------
# Button 1 Interrupt Disable
# Returns : Changes button state global
# ----------------------------------------------------------------------
def b1_disable():
	global button_state
	button_state &= ~1
	print("b1 disable")


# ----------------------------------------------------------------------
# Button 2 Interrupt Enable
# Returns : Changes button state global
# ----------------------------------------------------------------------
def b2_enable():
	global B2_GPIO, button_state
	GPIO.setup(B2_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	try:
		GPIO.add_event_detect(B2_GPIO, GPIO.RISING, callback=b2_release, bouncetime=200)
	except Exception:
		pass
	finally:
		signal.signal(signal.SIGINT, signal_handler)
		button_state &= ~2
		print("b2 enable")


# ----------------------------------------------------------------------
# Button 2 Interrupt Disable
# Returns : Changes button state global
# ----------------------------------------------------------------------
def b2_disable():
	global button_state
	button_state &= ~2
	print("b2 disable")


# ----------------------------------------------------------------------
# Button 3 Interrupt Enable
# Returns : Changes button state global
# ----------------------------------------------------------------------
def b3_enable():
	global B3_GPIO, button_state
	GPIO.setup(B3_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	try:
		GPIO.add_event_detect(B3_GPIO, GPIO.RISING, callback=b3_release, bouncetime=200)
	except Exception:
			pass
	finally:
		signal.signal(signal.SIGINT, signal_handler)
		button_state &= ~4
		print("b3 enable")


# ----------------------------------------------------------------------
# Button 3 Interrupt Disable
# Returns : Changes button state global
# ----------------------------------------------------------------------
def b3_disable():
	global button_state
	button_state &= ~4
	print("b3 disable")


# ----------------------------------------------------------------------
# Get Function for Buttons State
# Returns: State of Buttons
# (BIT0 = Button 1 | BIT1 = Button 2 |BIT2 = Button 3 )
# ----------------------------------------------------------------------
def get_button_state():
	global button_state
	return button_state


# ----------------------------------------------------------------------
# Description:
# Parameters: None
# Returns: None
# ----------------------------------------------------------------------
def pollButtons():
	# Initializes GPIO button for polling
	BTN_L = 14
	BTN_C = 15
	BTN_R = 18
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM)
	GPIO.setup([BTN_L, BTN_C, BTN_R], GPIO.IN, pull_up_down=GPIO.PUD_UP)

	# Waits until user presses button
	while GPIO.input(BTN_L) and GPIO.input(BTN_C) and GPIO.input(BTN_R):
		continue

	# Returns string for which button was pressed and waits until the user lets go
	if not GPIO.input(BTN_L):
		while not GPIO.input(BTN_L):
			continue
		return "left"
	elif not GPIO.input(BTN_C):
		while not GPIO.input(BTN_C):
			continue
		return "center"
	elif not GPIO.input(BTN_R):
		while not GPIO.input(BTN_R):
			continue
		return "right"
	else:
		return None

print("End of Bigfoot Init")
