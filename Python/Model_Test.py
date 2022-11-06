import unittest
import time
import Model as model


# ###### Debugging Test Example ######### #
class MyTestCase(unittest.TestCase):
    def test_something(self):
        self.assertEqual(True, False)  # add assertion here


# ###### (1) Debugging Test Code for Subject Tests ######### #
class BoardTests(unittest.TestCase):
    # 1.1 Tests Subject Board Functionality for Test #1 with LED
    def test_run_gpio_output(self):
        ser = model.open_serial()
        time.sleep(2)
        # Test LED Pin
        model.run_subject_test(23, 3, 6, 1, 1, ser)
        time.sleep(1)
        model.run_subject_test(23, 3, 6, 1, 0, ser)
        model.close_serial(ser)
        print("test")
        self.assertEqual(True, True, "Failed Test")

    # 1.2 Facade Tests Subject Board Functionality for Test #2
    def test_run_gpio_output_loading(self):

        # .encode([test], [pin], [instruction])
        s = model.crc_encode(0x03, 0x17, 0x81)

        ser = model.open_serial()
        time.sleep(2)
        model.subject_write(str_write=s, ser=ser)
        test_bytes = model.subject_read(ser_=ser)
        model.close_serial(ser)
        print(test_bytes)
        print(model.crc_decode(test_bytes, 0))      # type (Write / Read)
        print((model.crc_decode(test_bytes, 1)))    # pin #
        print((model.crc_decode(test_bytes, 2)))    # setting
        int_val = int.from_bytes(test_bytes, "big")
        print(int_val)

        self.assertEqual(True, True, "Failed Test")

    # 1.3 Tests Arduino Subject Board Pin Mapping
    def test_gpio_uno_facade(self):
        file = open('uno_facadeTest.config', 'r')
        lines = file.readlines()
        file_len = len(lines)

        loop_num = 1

        # Open Serial Comm
        ser = model.open_serial()
        time.sleep(2)

        while loop_num < file_len:
            conf = lines[loop_num].split(',')
            pin = conf[0]
            print(pin)
            res = conf[1].split('\n')
            print(res[0])

            # .encode([test], [pin], [instruction])
            s = model.crc_encode(0x01, int(pin), 0x81)
            model.subject_write(str_write=s, ser=ser)
            time.sleep(0.01)
            test_bytes = model.subject_read(ser_=ser)
            output = model.crc_decode(test_bytes, 1)

            self.assertEqual(True, float(res[0]) == float(output), "Failed Test | Pin Value: " + str(output))
            loop_num = loop_num + 1
            print(res[1])

        model.close_serial(ser)
        self.assertEqual(True, True, "Failed Test")

    # 1. Tests STM Nucleo Subject Board Pin Mapping
    def test_gpio_stm_facade(self):
        file = open('stm_facadeTest.config', 'r')
        lines = file.readlines()
        file_len = len(lines)

        loop_num = 1

        # Open Serial Comm
        ser = model.open_serial()
        time.sleep(2)

        while loop_num < file_len:
            conf = lines[loop_num].split(',')
            pin = conf[0]
            print(pin)
            res = conf[1].split('\n')
            print(res[0])

            # .encode([test], [pin], [instruction])
            s = model.crc_encode(0x01, int(pin), 0x81)
            model.subject_write(str_write=s, ser=ser)
            time.sleep(0.01)
            test_bytes = model.subject_read(ser_=ser)
            output = model.crc_decode(test_bytes, 1)

            self.assertEqual(True, float(res[0]) == float(output), "Failed Test | Pin Value: " + str(output))
            loop_num = loop_num + 1
            print(res[1])

        model.close_serial(ser)
        self.assertEqual(True, True, "Failed Test")


# ###### (2) Debugging Test Code for Serial ######### #
class SerialTests(unittest.TestCase):
    # 2.1 Tests that serial communication is established and subject board can reply
    def test_serial_basic(self):

        s = bytearray(3)
        s[0] = 0x01
        s[1] = 0x10
        s[2] = 0x03

        # s[3] = 0x0a
        # s[4] = 0x0D
        ser = model.open_serial()
        time.sleep(2)
        model.subject_write(str_write=s, ser=ser)
        #time.sleep(2)
        test_bytes = model.subject_read(ser_=ser)
        model.close_serial(ser)
        print(model.crc_decode(test_bytes, 0))

        print(test_bytes)
        int_val = int.from_bytes(test_bytes, "big")
        print(int_val)
        #time.sleep(2)
        # Pin ID Echos Back
        self.assertEqual(test_bytes[0], s[0], "Basic Communication to Subject Board Failed.")

    # 2.2 Tests Encoding CRC for data packet
    def test_crc_encode(self):

        s = bytearray(3)
        # .encode([test], [pin], [instruction])
        s = model.crc_encode(0x00, 0x01, 0x10)

        self.assertEqual(s[0], 0x01, "Failed to Encode CRC")
        self.assertEqual(s[1], 0x10, "Failed to Encode CRC")
        self.assertEqual(s[2], 0x03, "Failed to Encode CRC")

    # 2.3 Tests decoding CRC data packet to pin, result, and test id
    # E.G. 1 = pin, 2 = test, 3 = results
    def test_crc_decode(self):

        s = bytearray(3)
        s[0] = 0x01
        s[1] = 0x10
        s[2] = 0x03

        # controller.crc_decode([byte array], [value to return])
        # Returns pin value [Byte s[0]]
        output = model.crc_decode(s, 1)
        self.assertEqual(output, 0x01, "Failed to Decode CRC")

        # Returns result value [Byte s[1]]
        output = model.crc_decode(s, 3)
        self.assertEqual(output, 0x10, "Failed to Decode CRC")

        # Returns test value [Byte s[2]]
        output = model.crc_decode(s, 2)
        self.assertEqual(output, 0x00, "Failed to Decode CRC")


# ###### (3) Debugging Test Code for CLI ########## #
# CLI Functionality Tests
class CLITests(unittest.TestCase):

    # 3.1 Test that No Board Info is Obtained
    def test_board_list_None(self):
        board_out = model.board_list()
        print(board_out)
        # Test of Output (? Output "No Boards Detected" for no useful board)
        self.assertEqual(board_out, "No Boards Detected", "No board identification failed.")

    # 3.2 Test that Uno ID Info is Obtained
    def test_board_list_Uno(self):
        board_out = model.board_list()
        print(board_out)
        # Test of Output (? Output "Arduino Uno Detected" for Arduino Uno)
        self.assertEqual(board_out, "Arduino Uno Detected", "Arduino Uno Failed to be Detected.")

    # 3.3 Test that STM ID Info is Obtained
    def test_board_list_STM32F411(self):

        board_out = model.board_list()
        print(board_out)
        # Test of Output (? Output "STM32F411 Detected" for STM32F411)
        self.assertEqual(board_out, "STM32F411 Detected", "STM32F411 Failed to be Detected.")

    # ###################################################################
    # Additional Board Models can be added like below
    # 1) Add Board List ID Process
    # 2) Add Flash .bin file of board to subject_flash process
    # ###################################################################

    # 3.4 STM Board Addition Example
    def test_board_list_STM32F401(self):

        board_out = model.board_list()
        print(board_out)
        # Test of Output (? Output "STM32F401 Detected" for STM32F401)
        self.assertEqual(board_out, "STM32F401 Detected", "STM32F401 Failed to be Detected.")

    def test_board_list_STM32F446(self):

        board_out = model.board_list()
        print(board_out)
        # Test of Output (? Output "STM32F401 Detected" for STM32F401)
        self.assertEqual(board_out, "STM32F446 Detected", "STM32F446 Failed to be Detected.")

    # 3.5 Test that Two or More boards Info is Obtained
    def test_board_list_Overflow(self):

        board_out = model.board_list()
        print(board_out)
        # Test of Output (? Output "Overflow" for two+ board)
        self.assertEqual(board_out, "Overflow", "Two+ Boards Detected")

    # 3.6 Flash Subject Board Test
    def test_board_flash(self):

        board_out = model.board_list()
        model.subject_flash(board_out)

    # 3.7 USB Subject Board Test
    def test_usb(self):
        usb_out = model.usb_list()
        print("Filepath to USB: " + usb_out)


if __name__ == '__main__':
    unittest.main()
