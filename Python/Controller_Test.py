import unittest
import time
import Model as model


def pause():
    input("Press the <ENTER> key to continue...")


def Reset_Pins():
    model.bigfoot.set_mux_add(0, 0, 0)
    model.bigfoot.adc_enable(0)
    model.bigfoot.dac_enable(0)
    model.bigfoot.adc_load(0)


def DAC_Set(volt):
    model.bigfoot.dac_enable(1)
    Vout = volt * 788
    v1 = (int(Vout) & 0x0F00) >> 8
    v2 = (int(Vout) & 0x00FF)
    model.bigfoot.bus.write_word_data(0x0D, v1, v2)



# ##############################################################
# ################## (1) Subject Serial Calls ##################
# ###### Test # 1 | Output Test    ################# #
def Output_Test1(pin, enable, address, instruction):
    # Configure Bigfoot without load to adc
    model.bigfoot.high_current(0)
    model.bigfoot.low_current(0)
    model.bigfoot.adc_load(0)
    model.bigfoot.adc_enable(1)
    # State (on,off) | Enable # | Address #
    model.bigfoot.set_mux_add(1, enable, address)

    # .encode([test], [pin], [instruction])
    s = model.crc_encode(0x01, pin, instruction)
    ser = model.open_serial()
    time.sleep(2)

    model.subject_write(str_write=s, ser=ser)
    test_bytes = model.subject_read(ser_=ser)
    model.close_serial(ser)
    output = model.crc_decode(test_bytes, 2)

    # Read Bigfoot ADC
    time.sleep(0.02)
    if model.crc_decode(test_bytes, 0) == 0:
        adc = -1
    else:
        adc = model.bigfoot.rpi_i2c_adc()

    print(adc)


# ###### Test # 2 | Output w/ Load ################# #
def OutputLoad_Test2(pin, enable, address, instruction):
    # Configure Bigfoot with load
    model.bigfoot.high_current(0)
    model.bigfoot.low_current(0)
    model.bigfoot.adc_enable(1)
    model.bigfoot.adc_load(1)

    # State (on,off) | Enable # | Address #
    model.bigfoot.set_mux_add(1, enable, address)

    # .encode([test], [pin], [instruction])
    s = model.crc_encode(0x02, pin, instruction)
    ser = model.open_serial()
    time.sleep(2)
    model.subject_write(str_write=s, ser=ser)
    test_bytes = model.subject_read(ser_=ser)
    model.close_serial(ser)
    output = model.crc_decode(test_bytes, 2)

    # Read Bigfoot ADC
    time.sleep(0.02)
    if model.crc_decode(test_bytes, 0) == 0:
        adc = -1
    else:
        adc = model.bigfoot.rpi_i2c_adc()
    print(adc)


# ###### Test # 3 | Pull-Up Input  ################# #
def PullUp_Test3(pin, enable, address, instruction):
    # Configure Bigfoot w/
    model.bigfoot.high_current(0)
    model.bigfoot.low_current(0)
    model.bigfoot.adc_enable(1)
    model.bigfoot.adc_load(1)
    model.bigfoot.set_mux_add(1, enable, address)

    # .encode([test], [pin], [instruction])
    s = model.crc_encode(0x03, pin, instruction)
    ser = model.open_serial()
    time.sleep(2)
    model.subject_write(str_write=s, ser=ser)
    time.sleep(0.02)
    test_bytes = model.subject_read(ser_=ser)
    model.close_serial(ser)
    print(test_bytes)

    output = model.crc_decode(test_bytes, 2)
    print(output)
    time.sleep(0.01)
    adc = model.bigfoot.rpi_i2c_adc()
    print(adc)
    

# ###### Test # 3 | Pull-Up Input - No Load ################# #
def PullUp_Test3_NoLoad(pin, enable, address, instruction):
    # Configure Bigfoot w/
    model.bigfoot.high_current(0)
    model.bigfoot.low_current(0)
    model.bigfoot.adc_enable(1)
    model.bigfoot.adc_load(0)
    model.bigfoot.set_mux_add(1, enable, address)

    # .encode([test], [pin], [instruction])
    s = model.crc_encode(0x03, pin, instruction)
    ser = model.open_serial()
    time.sleep(2)
    model.subject_write(str_write=s, ser=ser)
    test_bytes = model.subject_read(ser_=ser)
    model.close_serial(ser)
    output = model.crc_decode(test_bytes, 2)
    time.sleep(0.1)
    adc = model.bigfoot.rpi_i2c_adc()
    print(adc)


# ###### Test # 4 | Pull-Down Input ################# #
def PullDown_Test4(pin, enable, address, instruction):
    # Configure Bigfoot w/
    model.bigfoot.adc_enable(1)
    model.bigfoot.adc_load(0)
    model.bigfoot.dac_enable(1)
    model.bigfoot.set_mux_add(1, enable, address)

    # FIX: Configure DAC
    model.bigfoot.rpi_i2c_dac()

    # .encode([test], [pin], [instruction])
    s = model.crc_encode(0x04, pin, instruction)
    ser = model.open_serial()
    time.sleep(2)
    model.subject_write(str_write=s, ser=ser)
    time.sleep(0.01)
    test_bytes = model.subject_read(ser_=ser)
    model.close_serial(ser)
    output = model.crc_decode(test_bytes, 2)
    time.sleep(0.1)
    adc = model.bigfoot.rpi_i2c_adc()
    print(adc)


# ###### Test # 4 | Pull-Down Input-NoLoad ################# #
def PullDown_Test4_NoLoad(pin, enable, address, instruction):
    # Configure Bigfoot w/
    model.bigfoot.adc_enable(1)
    model.bigfoot.adc_load(0)
    model.bigfoot.dac_enable(1)
    model.bigfoot.set_mux_add(1, enable, address)

    # FIX: Configure DAC
    model.bigfoot.rpi_i2c_dac()

    # .encode([test], [pin], [instruction])
    s = model.crc_encode(0x04, pin, instruction)
    ser = model.open_serial()
    time.sleep(2)
    model.subject_write(str_write=s, ser=ser)
    time.sleep(0.01)
    test_bytes = model.subject_read(ser_=ser)
    model.close_serial(ser)
    output = model.crc_decode(test_bytes, 2)
    time.sleep(0.01)
    adc = model.bigfoot.rpi_i2c_adc()
    print(adc)

# ###### Test # 5 & 6  | DAC Output ################# #
def DAC_Test(pin, enable, address, instruction, test):
    model.bigfoot.set_mux_add(0, 0, 0)
    model.bigfoot.adc_enable(0)
    model.bigfoot.dac_enable(1)
    model.bigfoot.adc_load(0)
    
    # Set DAC to first configuration instruction
    # .encode([test], [pin], [instruction])
    s = model.crc_encode(test, pin, instruction)
    ser = model.open_serial()
    time.sleep(2)
    model.subject_write(str_write=s, ser=ser)
    time.sleep(0.05)
    test_bytes = model.subject_read(ser_=ser)
    
    output = model.crc_decode(test_bytes, 3)

    model.bigfoot.set_mux_add(1, enable, address)

    # Configure Bigfoot to high logic
    model.bigfoot.rpi_i2c_dac()
    
    # Set DAC to first configuration instruction
    # .encode([test], [pin], [instruction])
    s = model.crc_encode(test, pin, instruction)
    model.subject_write(str_write=s, ser=ser)
    time.sleep(0.01)
    test_bytes = model.subject_read(ser_=ser)
    output = model.crc_decode(test_bytes, 3)
    model.close_serial(ser)
    print("ADC val: " + str(output))


# ###### Test # 7  | Set Power Mode ################# #
def PowerMode_Test7(pin, instruction):
    # Configure Bigfoot
    model.bigfoot.dac_enable(0)
    model.bigfoot.high_current(0)
    model.bigfoot.low_current(1)
    # Always On

    # Communication to Subject Serial
    # .encode([test], [pin], [instruction])
    s = model.crc_encode(0x07, pin, instruction)
    ser = model.open_serial()
    time.sleep(2)
    model.subject_write(str_write=s, ser=ser)
    #test_bytes = subject_read(ser_=ser)
    model.close_serial(ser)

    # Read Bigfoot Low Current Sensor
    # TIME DELAY
    time.sleep(0.2)
    current = model.bigfoot.rpi_i2c_ina219(1)
    print(current)


# ###### Test # 8  | Wake up    ################# #
def WakeUp_Test8(enable, address, instruction):
    # Configure Bigfoot
    # Set Wakeup pin
    model.bigfoot.set_mux_add(1, enable, address)
    model.bigfoot.dac_enable(1)
    model.bigfoot.high_current(0)
    model.bigfoot.low_current(1)

    # Configure Bigfoot to high logic
    model.bigfoot.set_vout(instruction)
    model.bigfoot.rpi_i2c_dac()

    # Red Bigfoot Low Current Sensor
    time.sleep(0.1)
    current = model.bigfoot.rpi_i2c_ina219(1)
    print(current)


# ##############################################################
# ##################### (2) Hardware Tests #####################

# ############## Validation Test 2.05.1      ################# #
def Validation_2051():

    l = input("Press the <y> to begin Test 2.05.1")
    Reset_Pins()
    model.bigfoot.low_current(0)
    model.bigfoot.adc_enable(1)
    model.bigfoot.adc_load(0)
    while l == "y":
        adc = model.bigfoot.rpi_i2c_adc()
        print(adc)
        l = input("Press the <y> to begin Test 2.05.1")
    # input("Press the <ENTER> to Exit")
    Reset_Pins()


# ############### Validation Test 2.06.1      ################# #
def Validation_2061():

    l = input("Press the <y> to begin Test 2.06.1")
    Reset_Pins()
    model.bigfoot.dac_enable(0)
    
    model.bigfoot.low_current(0)
    model.bigfoot.high_current(1)
    while l == "y":
        current = model.bigfoot.rpi_i2c_ina219(0)
        print(current)
        l = input("Press the <y> to begin Test 2.06.1")
    # input("Press the <ENTER> to Exit")
    Reset_Pins()


# ############## Validation Test 2.06.2      ################# #
def Validation_2062():

    l = input("Press the <y> to begin Test 2.06.2")
    Reset_Pins()
    model.bigfoot.dac_enable(0)
    model.bigfoot.high_current(0)
    model.bigfoot.low_current(1)
    while l == "y":
        current = model.bigfoot.rpi_i2c_ina219(1)
        print(current)
        l = input("Press the <y> to begin Test 2.06.2")
    # input("Press the <ENTER> to Exit")
    Reset_Pins()


# ##############################################################
# ################## (3) Integration Tests #####################
# ######## Validation Test 3.02.1    Output  ################# #
def Validation_3021(pin, e, a):
    input("Press the <ENTER> to begin Test 3.02.1")
    Output_Test1(pin, e, a, 0x0A)
    input("Press the <ENTER> to Exit")
    Reset_Pins()


# ###### Validation Test 3.03.1    Pull-Up     ################# #
def Validation_3031(pin, e, a):
    input("Press the <ENTER> to begin Test 3.04.1")
    PullUp_Test3(pin, e, a, 0x00)
    input("Press the <ENTER> to Exit")
    Reset_Pins()

# ###### Validation Test 3.06.1    Pull-Down ################# #
def Validation_3032(pin, e, a):
    input("Press the <ENTER> to begin Test 3.05.1")
    model.bigfoot.set_vout(3.3)
    PullDown_Test4(pin, e, a, 0)
    input("Press the <ENTER> to Exit")
    Reset_Pins()


# ###### Validation Test 3.06.1  Input Logic 3V3 ############## #
def Validation_3061_3V3_Logic(pin, e, a):
    input("Press the <ENTER> to begin Test 3.06.1")
    model.bigfoot.set_vout(0)
    print("DAC set to 0 V")
    DAC_Test(pin, e, a, 0x00, 0x05)
    pause()
    model.bigfoot.set_vout(0.3)
    print("DAC set to 0.3 V")
    DAC_Test(pin, e, a, 0x01, 0x05)
    pause()
    model.bigfoot.set_vout(0.6)
    print("DAC set to 0.6 V")
    DAC_Test(pin, e, a, 0x02, 0x05)
    pause()
    model.bigfoot.set_vout(1.5)
    print("DAC set to 1.5 V")
    DAC_Test(pin, e, a, 0x08, 0x05)
    pause()
    model.bigfoot.set_vout(1.8)
    print("DAC set to 1.8 V")
    DAC_Test(pin, e, a, 0x09, 0x05)
    pause()
    model.bigfoot.set_vout(3.3)
    print("DAC set to 3.3 V")
    DAC_Test(pin, e, a, 0x0A, 0x05)
    input("Press the <ENTER> to Exit")
    Reset_Pins()


# ###### Validation Test 3.06.1 & 3.07.1 Input Logic 3V3 ############## #
def Validation_3061_5V_Logic(pin, e, a):
    input("Press the <ENTER> to begin Test 3.06.1")
    model.bigfoot.set_vout(0)
    print("DAC set to 0 V")
    DAC_Test(pin, e, a, 0x00, 0x05)
    pause()
    model.bigfoot.set_vout(1)
    print("DAC set to 0.6 V")
    DAC_Test(pin, e, a, 0x02, 0x05)
    pause()
    model.bigfoot.set_vout(2.25)
    print("DAC set to 1.5 V")
    DAC_Test(pin, e, a, 0x08, 0x05)
    pause()
    model.bigfoot.set_vout(3.3)
    print("DAC set to 3.3 V")
    DAC_Test(pin, e, a, 0x0A, 0x05)
    input("Press the <ENTER> to Exit")
    Reset_Pins()
    model.bigfoot.set_vout(5)
    print("DAC set to 5 V")
    DAC_Test(pin, e, a, 0x09, 0x05)
    pause()


# ###### Validation Test 3.07.1 for 3.3 Logic ################# #
def Validation_3071_3V3_Logic(pin, e, a):
    input("Press the <ENTER> to begin Test 3.07.1")
    model.bigfoot.set_vout(3.3)
    print("DAC set to 3.3 V")
    DAC_Test(pin, e, a, 0x0A, 0x06)
    pause()
    model.bigfoot.set_vout(1.5)
    print("DAC set to 1.5 V")
    DAC_Test(pin, e, a, 0x08, 0x06)
    pause()
    model.bigfoot.set_vout(1)
    print("DAC set to 1 V")
    DAC_Test(pin, e, a, 0x05, 0x06)
    pause()
    model.bigfoot.set_vout(0)
    print("DAC set to 0 V")
    DAC_Test(pin, e, a, 0x00, 0x06)
    input("Press the <ENTER> to Exit")
    Reset_Pins()


# ###### Validation Test 3.08.1              ################# #
def Validation_3081(pin, instruction):
    Reset_Pins()
    model.serial_setup()
    input("Press the <ENTER> to begin Test 3.08.1")
    PowerMode_Test7(pin, instruction)
    Validation_2062()
    # input("Press the <ENTER> to Exit")
    Reset_Pins()


# ###### Validation Test 3.09.1              ################# #
def Validation_3091(pin, e, a):
    input("Press the <ENTER> to begin Test 3.09.1")
    WakeUp_Test8(e, a, 0)
    input("Press the <ENTER> to Exit")
    Reset_Pins()


# ##############################################################
# ################## (4) Software Tests ########################

# ###### Littlefoot Test                     ################# #
def Littlefoot_Test():
    input("Press the <ENTER> to begin Test Little-foot")
    DAC_Set(3.3)
    for i in range(1, 9):
        for j in range(8):
            print("Enable " + str(i) + " | Address " + str(j))
            model.bigfoot.set_mux_add(1, i, j)
            pause()
    print("Littlefoot Test")


# ###### Arduino Power Modes Test                     ################# #
def ArduinoSleep_Test():
    input("Press the <ENTER> to begin Test on Arduino Sleep Modes")
    model.bigfoot.set_vout(0)
    Validation_2062() 
    Validation_3081(2, 1)
    Validation_3091(61, 8, 6)  # Test 8 Wake up + (INA)
    Validation_2062() 
    Validation_3081(2, 2)
    Validation_3091(61, 8, 6)  # Test 8 Wake up + (INA)
    Validation_2062() 
    Validation_3081(2, 3)
    Validation_3091(61, 8, 6)  # Test 8 Wake up + (INA)
    Validation_2062() 
    Validation_3081(2, 4)
    Validation_3091(61, 8, 6)  # Test 8 Wake up + (INA)
    Validation_2062() 
    Validation_3081(2, 5)
    Validation_3091(61, 8, 6)  # Test 8 Wake up + (INA)
    Validation_2062() 
    Validation_3081(2, 6)
    Validation_3091(61, 8, 6)  # Test 8 Wake up + (INA)
    print("End of Sleep Mode Test")


# ##################### STM Power Modes Test    ################# #
def STM32Sleep_Test():
    Validation_2062() 
    input("Press the <ENTER> to begin Test on Arduino Sleep Modes")
    model.bigfoot.set_vout(3.3)
    Validation_3081(1, 1)
    Validation_3091(34, 5, 1)  # Test 8 Wake up + (INA)
    Validation_2062() 
    Validation_3081(2, 2)
    Validation_3091(34, 5, 1)  # Test 8 Wake up + (INA)
    Validation_2062() 
    Validation_3081(3, 3)
    Validation_3091(34, 5, 1)  # Test 8 Wake up + (INA)
    print("End of Sleep Mode Test")


if __name__ == '__main__':
    print("Run Validation Tests on Arduino")
    # Generic DAC Setting
    DAC_Set(3.3)
    Littlefoot_Test()

    # No Subject Board
    Validation_2051()                     # ADC Check
    Validation_2061()                     # LOW CURRENT
    Validation_2062()                     # HIGH CURRENT

    # Arduino Uno Subject Board Connected
    Validation_3021(23, 3, 6)             # Test 1 Output Logic (ADC)
    Validation_3031(23, 3, 6)             # Test 3 Pull Up (ADC Test)
    Validation_3031(23, 3, 6)             # Test 3 Pull Up (ADC Test)
    Validation_3031(21, 3, 4)             # Test 3 Pull Up (ADC Test)
    Validation_3031(32, 4, 7)             # Test 3 Pull Up (ADC Test)
    Validation_3061_5V_Logic(23, 3, 6)   # Test 5 Logic Levels (DAC Test)
    Validation_3071_3V3_Logic(34, 5, 1)    # Test 6 Subject ADC (DAC Test)
    Validation_3081(2, 1)                 # Test 7 Sleep Mode #2 + (INA)
    Validation_3091(61, 8, 6)             # Test 8 Wake up + (INA)

    ArduinoSleep_Test()

    # STM32 Nucleo Subject Board
    Validation_3032(21, 3, 4)            # Test 4 Pull Down (ADC Test)
    Validation_3032(23, 3, 6)
    
    PullUp_Test3_NoLoad(32, 4, 7, 0)
    PullDown_Test4_NoLoad(32, 4, 7, 0)
    PullUp_Test3(23, 3, 6, 0x00)
    PullDown_Test4(23, 3, 6, 0x00)
    Validation_3071_3V3_Logic(34, 5, 1)

    STM32Sleep_Test()

    
    


