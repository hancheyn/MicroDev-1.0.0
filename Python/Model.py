# ----------------------------------------------------------------------
# MicroDev Project: Controller
# Date: May 2, 2022
# Description: Model
# This module contains calls to the Raspbian OS CLI as well as the 
# subject testing process
# Authors:
# Nathan Hanchey
# Dylan Vetter
# Connor Inglet
# Corey Noura
# 
# Sources & References:
# www.geeksforgeeks.org
# https://medium.com
# https://controllerstech.com/low-power-modes-in-stm32/
# ----------------------------------------------------------------------

import time
import serial
import subprocess
import Bigfoot as bigfoot
import View as view

# STM32  ADC = 16 | GPIO/Pinout = 64 Pins | Sleep Modes = 3
# ARDUINO UNO  ADC = 6 | GPIO/Pinout = 38 Pins | Sleep Modes = 6
# from serial import to_bytes
# ******************************** Subject Board I/O ************************************
"""
Subject Board I/O
Serial Communication to and from Subject Development Board
"""


# ----------------------------------------------------------------------
# Description: This will open a serial port
# Accepts: NA
# Returns: serial port
# ----------------------------------------------------------------------
def open_serial():
    ser = serial.Serial('/dev/ttyACM0', 115200)
    ser.flushInput()
    ser.flushOutput()
    return ser


# ----------------------------------------------------------------------
# Description: Closes Serial Port
# Accepts: serial port
# Returns: NA
# ----------------------------------------------------------------------
def close_serial(ser):
    ser.close()


# ----------------------------------------------------------------------
# Description: Writes Serial 3 byte array to Subject Board
# Accepts: 3 byte array | open serial port
# Returns: NA
# ----------------------------------------------------------------------
def subject_write(str_write, ser):
    #ser.flushInput()
    #ser.flushOutput()
    try:
        ser.write(str_write)  # write a string
    except serial.SerialException as e:
        if e.errno == 13:
            raise e
        print("New Eeror")
        pass
    except OSError:
        print("New Eeror2")
        pass


# ----------------------------------------------------------------------
# Description:
# Accepts:
# Returns: data packet of 3 byte array
# ----------------------------------------------------------------------
def subject_read(ser_):
    try:
        # Read Byte array
        data = bytearray(3)
        # Set Timeout
        ser_.timeout = 0.5
        # READ SERIAL
        data = ser_.read(3)

    except serial.SerialException as e:
        if e.errno == 13:
            raise e
        print("Error in Serial")
        pass
    except OSError:
        print("Error in Serial")
        pass

    return data


# ----------------------------------------------------------------------
# CRC Decoding
# Description: CRC Fail
# Accepts: 3 byte array | which value to decode
# Returns: integer value of byte array
# ----------------------------------------------------------------------
def crc_decode(value, out_type):

    out = 0
    # Check CRC is correct first
    int_val = int.from_bytes(value, "big")
    if (int_val % 5) == 0:
        print("CRC Pass")
        out = 1
        # Conditional on which data point to decode
        # E.G. 1 = pin, 2 = test, 3 = results
        if out_type == 1:
            out = int(value[0])
            print("pin")
        elif out_type == 2:
            out = int(value[2])
            out = out & 0xF0
            out = out >> 4
            print("test")
        elif out_type == 3:
            out = int(value[1])
            # print("results")
    else:
        print("CRC Fail")

    # return type value
    return out


# ----------------------------------------------------------------------
# CRC Encoding
# Description: Encodes the Value with CRC encoding
# Accepts: Test ID # | Pin ID # | Instruction
# Returns: 3 byte array value
# ----------------------------------------------------------------------
def crc_encode(test, pin, instruction):

    # CRC KEY => 0101 = 5
    # 1) Find Remainder
    # byte array to int
    packet = bytearray(3)
    crc_byte = test << 4

    packet[0] = pin
    packet[1] = instruction
    packet[2] = crc_byte

    crc_data = int.from_bytes(packet, 'big')
    remainder = crc_data % 5

    # 2) Subtract Key - Remainder
    crc = 5 - remainder

    # 3) Add value to data
    crc_byte = crc_byte + crc

    # 4) int converts to Byte array
    packet[2] = crc_byte

    # return byte array
    return packet


# ----------------------------------------------------------------------
# Serial Setup
# Opens and Closes Serial to Double-Check Com.
# Return: (int) 1 == Connection Success | 0 == Failed Connection
# ----------------------------------------------------------------------
def serial_setup():

    s = bytearray(3)
    s[0] = 0x01
    s[1] = 0x10
    s[2] = 0x03

    ser = open_serial()
    time.sleep(2)
    subject_write(str_write=s, ser=ser)
    test_bytes = subject_read(ser_=ser)
    close_serial(ser)

    # print(len(test_bytes))
    print(test_bytes)
    if len(test_bytes) == 3:
        if test_bytes[0] == s[0] and test_bytes[1] == s[1] and test_bytes[2] == s[2]:
            val = 1
        else:
            val = 0
    else:
        val = -1

    print("serial setup complete")
    return val


# ******************************** Subject Tests ************************************
"""
Subject Test Processes

"""
# ----------------------------------------------------------------------
# GENERIC TEST FUNCTION
# Description: Runs test based on test and pin IDs
# Accepts: pin ID # | enable # | address # | enable ID # | instruction
# Returns: Test Results (voltage, logic, etc.)
# ----------------------------------------------------------------------
def run_subject_test(pin, enable, address, test, instruction, ser):

    # Conditional for Facade Test
    if instruction >= 64:
        print("facade")

    val = 0

    # Call Test function
    # Conditional for Test
    if test == 1:
        val = run_gpio_output_test(pin, enable, address, instruction, ser)
    elif test == 2:
        val = run_gpio_output_loading_test(pin, enable, address, instruction, ser)
    elif test == 3:
        val = run_gpio_input_pull_up_test(pin, enable, address, instruction, ser)
    elif test == 4:
        val = run_gpio_input_pull_down_test(pin, enable, address, instruction, ser)
    elif test == 5:
        val = run_gpio_input_logic_level_test(pin, enable, address, instruction, ser)
    elif test == 6:
        val = run_adc_test(pin, enable, address, instruction, ser)
    elif test == 7:
        val = run_power_mode_test(pin, instruction, ser)
    elif test == 8:
        val = run_wakeup_test(pin, enable, address, instruction)
       

    # return results for specified pin
    return val


# ----------------------------------------------------------------------
# Description: Test ID #1 | GPIO Output Test
# Send Logic Voltage as Parameter (sets logic based on BIT0 of instruction)
# HIGH = 1 | LOW = 0
# instruction BIT7 controls facade test
# Spec: 3.00 Output Logic
# Accepts: pin ID # | enable # | address # | enable ID # | instruction
# Returns: Bigfoot adc voltage
# ----------------------------------------------------------------------
def run_gpio_output_test(pin, enable, address, instruction, ser):

    # Configure Bigfoot without load to adc
    bigfoot.adc_load(0)
    bigfoot.adc_enable(1)
    # State (on,off) | Enable # | Address #
    bigfoot.set_mux_add(1, enable, address)

    # Communication to Subject Serial
    # Configures output low on subject board
    # .encode([test], [pin], [instruction])
    s = crc_encode(0x01, pin, instruction)
    subject_write(str_write=s, ser=ser)
    test_bytes = subject_read(ser_=ser)
    output = crc_decode(test_bytes, 2)

    # Read Bigfoot ADC
    time.sleep(0.01)
    if crc_decode(test_bytes, 0) == 0:
        adc = -1
    else:
        adc = bigfoot.rpi_i2c_adc()

    # default pin configurations
    bigfoot.adc_enable(0)
    bigfoot.set_mux_add(0, 0, 0)
    return adc


# ----------------------------------------------------------------------
# Description: Test ID #2 | GPIO Output Test /w Load
# Send Logic Voltage as Parameter (sets voltage based on BIT3-BIT0 of instruction)
# (Approximate Values) 0.311 = 0x01 | 0.619	= 0x02 | ... | 3.08	= 0x0A | ... | 4.59	= 0x0F
# instruction BIT7 controls facade test
# Spec: 3.02 Voltage Under Load
# Accepts: pin ID # | enable # | address # | enable ID # | instruction
# Returns: Bigfoot adc
# ----------------------------------------------------------------------
def run_gpio_output_loading_test(pin, enable, address, instruction, ser):
    # Configure Bigfoot with load
    bigfoot.adc_enable(1)
    bigfoot.adc_load(1)

    # State (on,off) | Enable # | Address #
    bigfoot.set_mux_add(1, enable, address)

    # Communication to Subject Serial
    # Configures output low on subject board
    # .encode([test], [pin], [instruction])
    s = crc_encode(0x02, pin, instruction)
    subject_write(str_write=s, ser=ser)
    test_bytes = subject_read(ser_=ser)
    output = crc_decode(test_bytes, 2)

    # Read Bigfoot ADC
    time.sleep(0.01)
    if crc_decode(test_bytes, 0) == 0:
        adc = -1
    else:
        adc = bigfoot.rpi_i2c_adc()

    # default pin configurations
    bigfoot.adc_enable(0)
    bigfoot.adc_load(0)
    bigfoot.set_mux_add(0, 0, 0)
    return adc


# ----------------------------------------------------------------------
# Description: Test ID #3 | Input Pull Up Resister Test
# instruction BIT7 controls facade test
# Spec: 3.03 Input Resistance Value & 3.04 Pull Up Test
# ADC Load Resistor
# Accepts: pin ID # | enable # | address # | enable ID # | instruction
# Returns: Bigfoot adc
# ----------------------------------------------------------------------
def run_gpio_input_pull_up_test(pin, enable, address, instruction, ser):

    # Configure Bigfoot w/
    bigfoot.adc_enable(1)
    bigfoot.adc_load(0)
    if instruction != 0x00:
        bigfoot.adc_load(1)

    bigfoot.set_mux_add(1, enable, address)

    # FIX: Configure DAC
    # bigfoot.dac_enable(1)
    # bigfoot.rpi_i2c_dac(instruction)

    # Communication to Subject Serial
    # Configure input pull-ups
    # Returns Subject Input Read
    # .encode([test], [pin], [instruction])
    instruction = 0x00
    s = crc_encode(0x03, pin, instruction)
    subject_write(str_write=s, ser=ser)
    test_bytes = subject_read(ser_=ser)
    output = crc_decode(test_bytes, 2)
    print(output)

    # Configure Bigfoot w/
    bigfoot.adc_enable(1)
    # Read Bigfoot ADC Voltage
    time.sleep(0.01)
    if crc_decode(test_bytes, 0) == 0:
        adc = -1
    else:
        adc = bigfoot.rpi_i2c_adc()

    # default pin configurations
    bigfoot.adc_enable(0)
    bigfoot.dac_enable(0)
    bigfoot.adc_load(0)
    bigfoot.set_mux_add(0, 0, 0)
    return adc


# ----------------------------------------------------------------------
# Description: Test ID #4 | Input Pull Down Test
# Send Voltage as Parameter (sets voltage based on BIT3-BIT0 of instruction)
# (Approximate Values) 0.311 = 0x01 | 0.619	= 0x02 | ... | 3.08	= 0x0A | ... | 4.59	= 0x0F
# instruction BIT7 controls facade test
# Spec: 3.03 Input Resistance Value & 3.05 Pull Down Test
# Configure DAC to Supply Voltage (To Test Internal Resistance)
# Accepts: pin ID # | enable # | address # | enable ID # | instruction
# Returns: Bigfoot adc
# ----------------------------------------------------------------------
def run_gpio_input_pull_down_test(pin, enable, address, instruction, ser):

    # Configure Bigfoot w/
    bigfoot.adc_enable(1)
    bigfoot.adc_load(0)
    bigfoot.set_mux_add(1, enable, address)

    # FIX: Configure DAC
    if instruction != 0:
        bigfoot.dac_enable(1)
        bigfoot.rpi_i2c_dac()

    # Communication to Subject Serial
    # Configure input pull-downs
    # Returns Subject Input Read
    # .encode([test], [pin], [instruction])
    s = crc_encode(0x04, pin, instruction)
    subject_write(str_write=s, ser=ser)
    time.sleep(0.01)
    test_bytes = subject_read(ser_=ser)
    output = crc_decode(test_bytes, 2)

    # Read Bigfoot ADC Voltage
    time.sleep(0.01)
    if crc_decode(test_bytes, 0) == 0:
        adc = -1
    else:
        adc = bigfoot.rpi_i2c_adc()

    # default pin configurations
    bigfoot.adc_enable(0)
    bigfoot.dac_enable(0)
    bigfoot.set_mux_add(0, 0, 0)
    return adc


# ----------------------------------------------------------------------
# Description: Test ID #5 | Input Logic Level Test
# Send Voltage as Parameter (sets voltage based on BIT3-BIT0 of instruction)
# (Approximate Values) 0.311 = 0x01 | 0.619	= 0x02 | ... | 3.08	= 0x0A | ... | 4.59	= 0x0F
# instruction BIT7 controls facade test
# Spec: 3.06 Input Logic Test
# Accepts: pin ID # | enable # | address # | enable ID # | instruction
# Returns: Bigfoot adc
# ----------------------------------------------------------------------
def run_gpio_input_logic_level_test(pin, enable, address, instruction, ser):

    # Reset/Configure Bigfoot to Low Logic
    bigfoot.dac_enable(1)
    bigfoot.adc_load(0)
    bigfoot.rpi_i2c_dac()

    # Init Pins | Communication to Subject Serial
    # .encode([test], [pin], [instruction])
    s = crc_encode(0x05, pin, instruction)
    subject_write(str_write=s, ser=ser)
    test_bytes = subject_read(ser_=ser)
    output = crc_decode(test_bytes, 2)

    bigfoot.set_mux_add(1, enable, address)

    # Configure Bigfoot to high logic
    bigfoot.rpi_i2c_dac()
    time.sleep(0.01)

    # Communication to Subject Serial
    # .encode([test], [pin], [instruction])
    s = crc_encode(0x05, pin, instruction)
    subject_write(str_write=s, ser=ser)
    test_bytes = subject_read(ser_=ser)
    output = crc_decode(test_bytes, 3)
    print("Logic: " + str(output))

    # ADC
    time.sleep(0.02)
    if crc_decode(test_bytes, 0) == 0:
        adc = -1
    else:
        adc = int(test_bytes[1])

    # default pin configurations
    bigfoot.dac_enable(0)
    bigfoot.set_mux_add(0, 0, 0)
    return adc


# ----------------------------------------------------------------------
# Description: Test ID #6 | ADC Test
# Send Voltage as Parameter (sets voltage based on BIT3-BIT0 of instruction)
# (Approximate Values) 0.311 = 0x01 | 0.619	= 0x02 | ... | 3.08	= 0x0A | ... | 4.59	= 0x0F
# instruction BIT7 controls facade test
# Spec: 3.07 ADC Test
# Pass in the voltage to use also
# Accepts: pin ID # | enable # | address # | enable ID # | instruction
# Returns: Subject adc
# ----------------------------------------------------------------------
def run_adc_test(pin, enable, address, instruction, ser):

    # Communication to Subject Serial to configure
    # Configure to ADC Input
    bigfoot.adc_enable(0)
    bigfoot.adc_load(0)
    bigfoot.set_mux_add(1, enable, address)

    # Configure Bigfoot to reset Subject ADC pins
    # Enable DAC
    bigfoot.dac_enable(1)
    bigfoot.rpi_i2c_dac()
    time.sleep(0.02)

    # Set DAC to first configuration instruction
    # .encode([test], [pin], [instruction])
    s = crc_encode(0x06, pin, instruction)
    subject_write(str_write=s, ser=ser)
    time.sleep(0.01)
    test_bytes = subject_read(ser_=ser)
    output = crc_decode(test_bytes, 3)
    print("ADC val: " + str(output))

    # Communication to Subject Serial to read ADC
    time.sleep(0.02)
    if crc_decode(test_bytes, 0) == 0:
        adc = -1
    else:
        adc = (int(output) << 2)
        # * 5.2) / 1024

    # default pin configurations
    bigfoot.dac_enable(0)
    bigfoot.set_mux_add(0, 0, 0)
    return adc


# ----------------------------------------------------------------------
# Description: Test ID #7 | Power Mode Test
# Sleep Mode sent as 'instruction' to serial com.
# instruction BIT7 controls facade test
# Wakeup Pin Set through 'pin' ID
# Spec: 3.08 Power Modes
# Accepts: pin ID # | instruction (sleep_mode)
# Returns: Bigfoot current
# ----------------------------------------------------------------------
def run_power_mode_test(pin, instruction, ser):

    # Configure Bigfoot
    bigfoot.dac_enable(0)
    bigfoot.high_current(0)
    bigfoot.low_current(1)
    # Always On

    # Communication to Subject Serial
    # .encode([test], [pin], [instruction])
    s = crc_encode(0x07, pin, instruction)
    subject_write(str_write=s, ser=ser)


    # Read Bigfoot Low Current Sensor
    # TIME DELAY
    time.sleep(0.2)
    current = bigfoot.rpi_i2c_ina219(1)

    # default pin configurations
    bigfoot.dac_enable(0)
    bigfoot.set_mux_add(0, 0, 0)
    return current


# ----------------------------------------------------------------------
# Description: Test ID #8 | Runs wake up test
# Accepts: pin ID # | enable # | address # | enable ID # | instruction
# Spec: 3.08 Wakeup
# Returns: Bigfoot current
# ----------------------------------------------------------------------
def run_wakeup_test(pin, enable, address, instruction):
    # Configure Bigfoot
    # Set Wakeup pin
    bigfoot.adc_enable(0)
    bigfoot.adc_load(0)
    bigfoot.set_mux_add(1, enable, address)
    bigfoot.dac_enable(1)
    bigfoot.high_current(0)
    bigfoot.low_current(1)
    

    # Configure Bigfoot to high logic
    bigfoot.set_vout(0)
    bigfoot.rpi_i2c_dac()
    # TIME DELAY?
    time.sleep(0.1)
    bigfoot.set_vout(3.3)
    bigfoot.rpi_i2c_dac()
    # Read Bigfoot Low Current Sensor
    time.sleep(0.1)
    current = bigfoot.rpi_i2c_ina219(1)

    # default pin configurations
    bigfoot.set_mux_add(0, 0, 0)
    bigfoot.dac_enable(0)
    return current


def current_read():
    bigfoot.high_current(0)
    bigfoot.low_current(1)
    current = bigfoot.rpi_i2c_ina219(1)
    return current


# ******************************** Command Line Interface ************************************
"""
Command Line Interface 
The following function methods deal with the linux command line interface.
These instructions are used alongside the arduino-cli and stm-link applications
"""


# ----------------------------------------------------------------------
# Description: Flashes the Subject Board
# Accepts: board detected string (from board_list function)
# Returns: NA
# ----------------------------------------------------------------------
def subject_flash(board):
    res = "error"
    tloop = 0
    
    # ARDUINO UNO FLASH
    if board == "Arduino Uno Detected":
        res = subprocess.getstatusoutput(
            f'arduino-cli upload -b arduino:avr:uno -p /dev/ttyACM0 -i Flash/serial_com.ino.with_bootloader.bin')
        # Confirm Success with CLI output

    # STM32F411 FLASH
    elif board == "STM32F411 Detected":
        reflash = True
        while reflash and tloop < 5:
            tloop = tloop + 1
            res = subprocess.getstatusoutput(
                    f'st-flash write Flash/stmf411.bin 0x08000000')
            s = len(res[1])
            if s < 870:
                reflash = False
                
    # STM32F401 FLASH
    elif board == "STM32F401 Detected":
        reflash = True
        while reflash and tloop < 5:
            tloop = tloop + 1
            res = subprocess.getstatusoutput(
                f'st-flash write Flash/stmf401.bin 0x08000000')
            s = len(res[1])
            if s < 870:
                reflash = False

    # STM32F446 FLASH
    elif board == "STM32F446 Detected":
        reflash = True
        while reflash and tloop < 5:
            tloop = tloop + 1
            res = subprocess.getstatusoutput(
                f'st-flash write Flash/stmf446.bin 0x08000000')
            s = len(res[1])
            if s < 870:
                reflash = False
        print(s)

    """
    Add new subject board flash conditional...
    
    """
    
    print(res)
    err_check = str(res[1]).find('error')
    if err_check < 0:
        return False
    return True


# ----------------------------------------------------------------------
# Description: Uses CLI to read for subject board connections
# Accepts: NA
# Returns: [string] board type
# ----------------------------------------------------------------------
def board_list():

    # 1) ARDUINO DETECTION
    res_ardiuno = subprocess.getstatusoutput(f'arduino-cli board list')

    # PARSE DATA FOR ARDUINO
    ARD = res_ardiuno[1].split("\n")
    boards = len(ARD)

    # No Board Detection
    if boards < 2:
        return "No Boards Detected"

    elif boards == 2:
        print("Internet Disconnected")

    # Arduino Uno Detection
    elif boards == 3 or boards == 4:
        board_type = ARD[1].split(" ")
        # print(board_type[4])
        if board_type[4] == "Serial":
            # print(board_type[8])
            if board_type[8] == "Uno":
                return "Arduino Uno Detected"
        elif board_type[4] == "Unknown":
            print(board_type)

    else:
        return "Overflow"

    
    # 2) STM DETECTION
    res_stm = subprocess.getstatusoutput(f'st-info --probe')
    # PARSE DATA FOR STM'S
    # print(res_stm[1])
    if len(res_stm) > 1:
        STM = res_stm[1].split("\n")
        boards1 = len(STM)
    else:
        boards1 = 0
        
    # STM UNIQUE IDs
    # 0x0433: STM32F401xD/E
    # 0x0431: STM32F411xC/E
    # 0x0421: STM32F446
    if boards1 > 6:

        Chip_ID = STM[5].split(" ")
        Chip_ID = list(filter(None, Chip_ID))

        Family = STM[6].split(" ")
        Family = list(filter(None, Family))
        Family = str(Family[1]).replace("x", "")

        print(Chip_ID)
        print(Family)
        if Chip_ID[1] == "0x0431":
            return "STM32F411 Detected"

        elif Chip_ID[1] == "0x0433":
            return "STM32F401 Detected"

        elif Family == "F446" and Chip_ID[1] == "0x0421":
            return "STM32F446 Detected"

    #  3) ADD NEW SUBJECT BOARD HERE
    """
    The Process for adding a new subject board in software...
    
    """

    # Use class globals for board file path and id info
    return "No Boards Detected"


# ----------------------------------------------------------------------
# Description: Loops board_list function until subject board is detected
# Accepts: NA
# Returns: (string) type of board
# ----------------------------------------------------------------------
def board_wait():
    cont = 1
    waiti = 0
    while cont == 1:
        
        _board_type = board_list()
        state_buttons = bigfoot.get_button_state()
        print("Button State: " + str(state_buttons))
        if state_buttons & 2 == 2:
            bigfoot.b2_disable()
            view.setShutdownScreen()
            shutdown()

        if _board_type == "No Boards Detected" or _board_type == "Overflow":
            cont = 1
            if check_5V() > 4.5:
                waiti = waiti + 1
                if waiti > 2:
                    cont = 0
            else:
                waiti = 0
        else:
            cont = 0
            print(_board_type)

    return _board_type


# ----------------------------------------------------------------------
# Description: Uses CLI to read for subject board connections
# Accepts: NA
# Returns: [string] usb file path or "None"
# ----------------------------------------------------------------------
def usb_list():

    # CLI call
    user = subprocess.getstatusoutput(f'whoami')
    mess = "ls /media/" + str(user[1])
    res_usb = subprocess.getstatusoutput(mess)
    print(res_usb)
    
    if "NOD" in res_usb[1]:
        split_usb = str(res_usb).split('\\')
        split_usb = str(split_usb[0]).split('\'')
        print(split_usb)
    else:
        split_usb = res_usb

    # Find USB Drives (Not STM Board)
    for i in split_usb:
        if i != 0:
            if "NOD" not in i and "(" not in i:
                print(i)
                return "/media/" + str(user[1]) + "/" + i

    print("usb lists")
    return "None"


# ----------------------------------------------------------------------
# Description: Uses CLI to Write File to usb
# Accepts: detailed_array
# Returns: N/A
# ----------------------------------------------------------------------
def usb_save(detailed_array):
    usb_filepath = usb_list()
    if usb_filepath != "None":
        usb_file = usb_filepath
        usbf = open("MicroDevTest_Results.txt", "w")
        for i in detailed_array:
            usbf.write(i + "\n")
        usbf.close()
        h = subprocess.getstatusoutput("cp MicroDevTest_Results.txt " + usb_file)
        # Debug Debug_MicroDevTest.txt
        subprocess.getstatusoutput("cp Debug_MicroDevTest.txt " + usb_file)
        
    try:
        if usb_filepath == "None":
            return "None"
        
        usbt = open(usb_file + "/MicroDevTest_Results.txt", "r")
        usbt.close()
        return "Success"
        
    except Exception:
        print("Could Not Open USB File")
        pass
    
    return "Fail"
        

# ----------------------------------------------------------------------
# Description: Shutdown Instruction
# Accepts: N/A
# Returns: N/A
# ----------------------------------------------------------------------
def shutdown():
    subprocess.getstatusoutput(f'sudo shutdown now')
    #print("shutdown attempt")


# ----------------------------------------------------------------------
# Description: 5V Pin ADC Check
# Accepts: N/A
# Returns: Pin Voltage Output
# ----------------------------------------------------------------------
def check_5V():
    bigfoot.dac_enable(0)
    bigfoot.adc_load(0)
    bigfoot.adc_enable(1)
    bigfoot.set_mux_add(1, 4, 3)
    time.sleep(0.01)
    v = bigfoot.rpi_i2c_adc()
    return v
    

# ----------------------------------------------------------------------
# Description: 3V3 Pin ADC Check
# Accepts: N/A
# Returns: Pin Voltage Output
# ----------------------------------------------------------------------
def check_3V3():
    bigfoot.dac_enable(0)
    bigfoot.adc_load(0)
    bigfoot.adc_enable(1)
    bigfoot.set_mux_add(1, 3, 1)
    time.sleep(0.01)
    v = bigfoot.rpi_i2c_adc()
    return v
    
