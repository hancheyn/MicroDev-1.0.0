import unittest
import time
import Bigfoot as bigfoot
import subprocess


# ###### 1 Debugging Test Example ######### #
class GPIOTest(unittest.TestCase):
    # 1.1 GPIO Integration test
    def test_GPIO(self):
        
        # Mux Address
        bigfoot.set_mux_add(0, 1, 7)
        time.sleep(1)
        bigfoot.set_mux_add(1, 0, 1)
        time.sleep(1)
        bigfoot.set_mux_add(1, 0, 2)
        time.sleep(1)
        bigfoot.set_mux_add(1, 0, 3)
        time.sleep(1)
        bigfoot.set_mux_add(1, 0, 4)
        time.sleep(1)
        bigfoot.set_mux_add(1, 0, 5)
        time.sleep(1)
        bigfoot.set_mux_add(1, 0, 6)
        time.sleep(1)
        bigfoot.set_mux_add(1, 0, 7)
        time.sleep(1)
        
        # Enables Test
        bigfoot.set_mux_add(1, 1, 0)
        time.sleep(1)
        bigfoot.set_mux_add(1, 2, 0)
        time.sleep(1)
        bigfoot.set_mux_add(1, 3, 0)
        time.sleep(1)
        bigfoot.set_mux_add(1, 4, 0)
        time.sleep(1)
        bigfoot.set_mux_add(1, 5, 0)
        time.sleep(1)
        bigfoot.set_mux_add(1, 6, 0)
        time.sleep(1)
        bigfoot.set_mux_add(1, 7, 0)
        time.sleep(1)
        bigfoot.set_mux_add(1, 8, 0)
        time.sleep(1)
        bigfoot.set_mux_add(0, 1, 0)
        
        # Others
        bigfoot.low_current(1)
        time.sleep(1)
        bigfoot.low_current(0)
        
        bigfoot.high_current(1)
        time.sleep(1)
        bigfoot.high_current(0)
        
        bigfoot.adc_enable(1)
        time.sleep(1)
        bigfoot.adc_enable(0)
        
        bigfoot.adc_load(1)
        time.sleep(1)
        bigfoot.adc_load(0)
        
        bigfoot.dac_enable(1)
        time.sleep(1)
        bigfoot.dac_enable(0)
        
        self.assertEqual(True, True)  # add assertion here
        
    # 1.2 I2C Integration test
    def test_I2C(self):
        subprocess.getstatusoutput("sudo i2cdetect -y 1")
        print("I2C Test")


if __name__ == '__main__':
    unittest.main()
