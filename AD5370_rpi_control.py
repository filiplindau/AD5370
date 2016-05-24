"""
    Control of the AD5370 EVAL board using a Raspberry Pi 3.
    Uses SPI and some GPIO pins.

    Created: 2016-05-12
    Author: Filip Lindau
"""

import spidev
import time
import RPi.GPIO as gpio
import struct


class AD5370_control(object):
    def __init__(self, port=0, device=0):
        self.V_ref = 5.00
        self.offsets = []
        self.gains = []
        self.voltages = []
        self.V_max = []
        self.V_min = []
        for o in range(40):
            self.offsets.append(0x1555)
            self.gains.append(1)
            self.V_max.append(4*self.V_ref*(65535-4*self.offsets[0])/2**16)
            self.V_min.append(4*self.V_ref*(-4*self.offsets[0])/2**16)
            self.voltages.append(0.0)

        self.LDAC_pin = 17
        self.BUSY_pin = 22
        self.CLR_pin = 27
        self.RESET_pin = 4

        self.spi = None
        self.spi_port = port       # There is one spi port exposed on the raspberry 40 pin connector
        self.spi_device = device     # There are two devices available through the spio cs pins

        self.clear_state = False
        self.setup_gpio()
        self.setup_spi()

    def setup_gpio(self):
        """
        Setup gpio pins on the raspberry. We use BCM naming.
        :return:
        """
        gpio.setmode(gpio.BCM)
        gpio.setup(self.LDAC_pin, gpio.OUT)
        gpio.setup(self.RESET_pin, gpio.OUT)
        gpio.setup(self.CLR_pin, gpio.OUT)
        gpio.setup(self.BUSY_pin, gpio.IN)

        self.reset()
        self.clear(False)

    def setup_spi(self):
        """
        Setup SPI interface on the raspberry using the spidev module.
        :return:
        """
        self.spi = spidev.SpiDev()
        self.spi.open(self.spi_port, self.spi_device)
        self.spi.mode = 0b01        # Something something polarity clock

    def close(self):
        self.spi.close()
        gpio.cleanup()

    def reset(self):
        gpio.output(self.RESET_pin, 0)
        time.sleep(0.001)
        gpio.output(self.RESET_pin, 1)

    def clear(self, enable=None):
        """
        Puts the device in CLEAR state (all outputs to ground).

        :param enable: True = Device in CLEAR state
         False = Device released from CLEAR
         None (default) = Toggle CLEAR state
        :return:
        """
        if enable is None:
            if self.clear_state is True:
                enable = False
            else:
                enable = True
        if enable is True:
            gpio.output(self.CLR_pin, 0)
            self.clear_state = True
        else:
            gpio.output(self.CLR_pin, 1)
            self.clear_state = False

    def load_dac(self, keep=False):
        """
        Updates DAC outputs by pulsing LDAC low. If keep=True the LDAC
        remains low, causing immediate updates when new values are written
         to the DAC.
        :param keep: False (default) = LDAC is pulsed causing written values to update the outputs.
        If new values are written after this, they are withheld until a new call to load_dac is made.
        True = LDAC is kept low, causing new values to immediately be output
        :return:
        """
        gpio.output(self.LDAC_pin, 0)
        if keep is False:
            time.sleep(0.001)
            gpio.output(self.LDAC_pin, 1)

    def write_value_int(self, output, value, immediate=True):
        """
        Write DAC value to specific output pin
        :param output: Pin to output. 0-39
        :param value:  Value to write. 0-65535
        :param immediate: True = Output voltage to pin immediately (default)
                          False= Wait for call to load_dac
        :return:
        """
        x = 0b11000000
        if not isinstance(output, int):
            raise TypeError('Output must be integer')
        if output < 0 or output > 39:
            raise ValueError('Output must be in range 0-39')
        if not isinstance(value, int):
            raise TypeError('Value must be integer')
        if value < 0 or value > 65535:
            raise ValueError('Value must be in range 0-65535')
        gr = output / 8     # Group number
        ch = output % 8     # Channel number
        a = 8*(gr+1) + ch
        self.voltages[output] = 4*self.V_ref*(value-4*self.offsets[output])/2**16
        d = struct.pack('<H', value)
        write_list = [x+a, ord(d[1]), ord(d[0])]
        retval = self.spi.xfer(write_list)
        if immediate is True:
            self.load_dac()

    def write_value_volt(self, output, value, immediate=True):
        """
        Write DAC value to specific output pin
        :param output: Pin to output. 0-39
        :param value:  Voltage to write. Range -6.67 - 13.3 V for default offset and 5 V reference
        :param immediate: True = Output voltage to pin immediately (default)
                          False= Wait for call to load_dac
        :return:
        """
        x = 0b11000000
        if not isinstance(output, int):
            raise TypeError('Output must be integer')
        if output < 0 or output > 39:
            raise ValueError('Output must be in range 0-39')
        if value < self.V_min[output] or value > self.V_max[output]:
            raise ValueError(''.join(('Value must be in range ', str(self.V_min[output]), '-', str(self.V_max[output]))))
        gr = output / 8     # Group number
        ch = output % 8     # Channel number
        a = 8*(gr+1) + ch
        self.voltages[output] = value
        dac = int(value * 2**16 / (4 * self.V_ref) + self.offsets[output] * 4)
        d = struct.pack('<H', dac)
        write_list = [x+a, ord(d[1]), ord(d[0])]
        self.spi.xfer(write_list)
        if immediate is True:
            self.load_dac()

    def write_offset_int(self, output, value):
        """
        Write DAC offset value to specific output pin
        :param output: Pin to output. 0-39
        :param value:  Value to write. 0-16384
        :return:
        """
        if not isinstance(output, int):
            raise TypeError('Output must be integer')
        if output < 0 or output > 39:
            raise ValueError('Output must be in range 0-39')
        if not isinstance(value, int):
            raise TypeError('Value must be integer')
        if value < 0 or value > 16384:
            raise ValueError('Value must be in range 0-65535')
        x = 0b10000000
        a = output
        d = struct.pack('<H', value)
        write_list = [x + a, ord(d[1]), ord(d[0])]
        self.spi.xfer(write_list)
        self.load_dac()
        self.offsets[output] = value
        self.V_max[output] = 4 * self.V_ref * (65535 - 4 * self.offsets[output]) / 2 ** 16
        self.V_min[output] = 4 * self.V_ref * (-4 * self.offsets[output]) / 2 ** 16
        self.voltages[output] = 4 * self.V_ref * (self.voltages[output] - 4 * self.offsets[output]) / 2 ** 16

    def write_offset_volt(self, output, value):
        """
        Write DAC offset value to specific output pin
        :param output: Pin to output. 0-39
        :param value:  Voltage offset to write. Range 0 - 20 V for 5 V reference
        :return:
        """
        if not isinstance(output, int):
            raise TypeError('Output must be integer')
        if output < 0 or output > 39:
            raise ValueError('Output must be in range 0-39')
        if not isinstance(value, int):
            raise TypeError('Value must be integer')
        if value < 0 or value > 20:
            raise ValueError('Value must be in range 0-20')
        x = 0b10000000
        gr = output / 8     # Group number
        ch = output % 8     # Channel number
        a = 8*(gr+1) + ch
        dac = int(value * 2**16 / (4 * self.V_ref * 4))
        d = struct.pack('<H', dac)
        write_list = [x + a, ord(d[1]), ord(d[0])]
        self.spi.xfer(write_list)
        self.load_dac()
        self.offsets[output] = dac
        self.V_max[output] = 4 * self.V_ref * (65535 - 4 * self.offsets[output]) / 2 ** 16
        self.V_min[output] = 4 * self.V_ref * (-4 * self.offsets[output]) / 2 ** 16
        self.voltages[output] = 4 * self.V_ref * (self.voltages[output] - 4 * self.offsets[output]) / 2 ** 16

    def write_gain(self, output, value):
        """
        Write DAC gain value to specific output pin
        :param output: Pin to output. 0-39
        :param value:  Value to write. 0-65535
        :return:
        """
        if not isinstance(output, int):
            raise TypeError('Output must be integer')
        if output < 0 or output > 39:
            raise ValueError('Output must be in range 0-39')
        if not isinstance(value, int):
            raise TypeError('Value must be integer')
        if value < 0 or value > 65535:
            raise ValueError('Value must be in range 0-65535')
        x = 0b01000000
        a = output
        d = struct.pack('<H', value)
        write_list = [x + a, ord(d[1]), ord(d[0])]
        self.spi.xfer(write_list)
        self.load_dac()

    def write_function(self, function, value):
        """
        Write special function
        :param function: 6 bit function code
        :param value:  Value to write. 0-65535
        :return:
        """
        if not isinstance(function, int):
            raise TypeError('Function must be integer')
        if function < 0 or function > 64:
            raise ValueError('Function must be in range 0-63')
        if not isinstance(value, int):
            raise TypeError('Value must be integer')
        if value < 0 or value > 65535:
            raise ValueError('Value must be in range 0-65535')
        x = 0b00000000
        a = function
        d = struct.pack('<H', value)
        write_list = [x + a, ord(d[1]), ord(d[0])]
        self.spi.xfer(write_list)


if __name__ == '__main__':
    ac = AD5370_control()