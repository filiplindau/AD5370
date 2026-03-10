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
import logging

logger = logging.getLogger()


class AD5370_control(object):
    def __init__(self, port=0, device=0):
        self.V_ref = 5.00
        self.offset_code = 0x1555       # Global offset dac value
        self.offsets = []
        self.gains = []
        self.input_codes = []
        self.V_max = []
        self.V_min = []
        for o in range(40):
            self.offsets.append(2**15)
            self.gains.append(2**16 - 1)
            self.V_max.append(4*self.V_ref*(65535-4*self.offset_code)/2**16)
            self.V_min.append(4*self.V_ref*(-4*self.offset_code)/2**16)
            self.input_codes.append(self.offset_code)

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

    def set_vref(self, value):
        self.V_ref = value
        for i, v in enumerate(self.input_codes):
            self.write_value_volt(i, v, immediate=False)
        self.load_dac()

    def write_value_int(self, channel, value, immediate=True):
        """
        Write DAC value to specific output pin
        :param channel: Pin to output. 0-39
        :param value:  Value to write. 0-65535
        :param immediate: True = Output voltage to pin immediately (default)
                          False= Wait for call to load_dac
        :return:
        """
        x = 0b11000000
        if not isinstance(channel, int):
            raise TypeError('Output must be integer')
        if channel < 0 or channel > 39:
            raise ValueError('Output must be in range 0-39')
        if not isinstance(value, int):
            raise TypeError('Value must be integer')
        if value < 0 or value > 65535:
            raise ValueError('Value must be in range 0-65535')
        gr = channel / 8     # Group number
        ch = channel % 8     # Channel number
        a = int(8*(gr+1) + ch)
        self.input_codes[channel] = value
        d = struct.pack('<H', value)
        write_list = [int(x + a), d[1], d[0]]
        logger.info(f"value: {value}, write_list: {write_list}")

        retval = self.spi.xfer2(write_list)
        if immediate is True:
            self.load_dac()

    def write_value_volt(self, channel, value, immediate=True):
        """
        Write DAC value to specific output pin
        :param channel: Pin to output. 0-39
        :param value:  Voltage to write. Range -6.67 - 13.3 V for default offset and 5 V reference
        :param immediate: True = Output voltage to pin immediately (default)
                          False= Wait for call to load_dac
        :return:
        """
        if not isinstance(channel, int):
            raise TypeError('Output must be integer')
        if channel < 0 or channel > 39:
            raise ValueError('Output must be in range 0-39')
        # if value < self.V_min[channel] or value > self.V_max[channel]:
        #     raise ValueError(''.join(('Value must be in range ', str(self.V_min[channel]), '-', str(self.V_max[channel]))))
        dac_code = value * 2**16 / (4 * self.V_ref) + 4 * self.offset_code
        input_code = int((dac_code - self.offsets[channel] + 2**15) * 2**16 / (self.gains[channel] + 1))
        logger.info(f"Channel {channel}:\n"
                    f"    vout     = {value:.3f}\n"
                    f"    dac_code = {dac_code}\n"
                    f"    dac_value= {input_code}")
        self.write_value_int(channel, int(dac_code), immediate=immediate)

    def get_output_volt(self, channel):
        dac_code = self.input_codes[channel] * (self.gains[channel] + 1) / 2**16 + self.offsets[channel] - 2**15
        dac_code = self.input_codes[channel]
        vout = 4 * self.V_ref * (dac_code - 4 * self.offset_code) / 2**16
        logger.debug(f"Channel {channel}:\n"
                    f"    vout     = {vout:.3f}\n"
                    f"    dac_code = {dac_code}\n"
                    f"    dac_value= {self.input_codes[channel]}")
        return vout

    def get_output_int(self, channel):
        return self.input_codes[channel]

    def write_offset_int(self, channel, value, immediate=True):
        """
        Write DAC offset value to specific output pin
        :param channel: Pin to output. 0-39
        :param value:  Value to write. 0-16384
        :return:
        """
        if not isinstance(channel, int):
            raise TypeError('Output must be integer')
        if channel < 0 or channel > 39:
            raise ValueError('Output must be in range 0-39')
        if not isinstance(value, int):
            raise TypeError('Value must be integer')
        if value < 0 or value > 65536:
            logger.exception(f"Offeset {channel}: {value}")
            raise ValueError('Value must be in range 0-65535')
        x = 0b10000000
        gr = channel / 8  # Group number
        ch = channel % 8  # Channel number
        a = int(8 * (gr + 1) + ch)

        d = struct.pack('<H', value)
        write_list = [int(x + a), d[1], d[0]]
        self.spi.xfer2(write_list)
        self.offsets[channel] = value
        logger.debug(f"channel {channel} offset raw {value}")
        dac_code_min = value - 2**15
        dac_code_max = self.gains[channel] + 1 + value - 2**15
        self.V_max[channel] = 4 * self.V_ref * (dac_code_max - 4 * self.offset_code) / 2 ** 16
        self.V_min[channel] = 4 * self.V_ref * (dac_code_min - 4 * self.offset_code) / 2 ** 16
        #self.input_codes[channel] = 4 * self.V_ref * (self.input_codes[channel] - 4 * self.offsets[channel]) / 2 ** 16
        if immediate is True:
            self.load_dac()

    def write_offset_volt(self, channel, value, immediate=True):
        """
        Write DAC offset value to specific output pin
        :param channel: Pin to output. 0-39
        :param value:  Voltage offset to write. Range 0 - 20 V for 5 V reference
        :return:
        """
        dac_code = value * 2**16 / (4 * self.V_ref)
        offset = int(dac_code + 2**15)
        logger.info(f"Write offset {channel}: {value}, {offset}")
        self.write_offset_int(channel, offset, immediate=immediate)

    def get_offset_volt(self, channel):
        offset_dac = self.offsets[channel]
        offset_v = (offset_dac - 2**15) * 4 * self.V_ref / 2**16
        logger.debug(f"Read offset {channel}: dac {offset_dac}, volt {offset_v}")
        return offset_v

    def get_offset_dac(self, channel):
        return self.offsets[channel]

    def write_gain_int(self, channel, value, immediate=True):
        """
        Write DAC gain value to specific output pin
        :param channel: Pin to output. 0-39
        :param value:  Value to write. 0-65535
        :return:
        """
        if not isinstance(channel, int):
            raise TypeError('Output must be integer')
        if channel < 0 or channel > 39:
            raise ValueError('Output must be in range 0-39')
        if not isinstance(value, int):
            raise TypeError('Value must be integer')
        if value < 0 or value > 65535:
            raise ValueError('Value must be in range 0-65535')
        x = 0b01000000
        gr = channel / 8  # Group number
        ch = channel % 8  # Channel number
        a = int(8 * (gr + 1) + ch)

        d = struct.pack('<H', value)
        write_list = [int(x + a), d[1], d[0]]
        self.spi.xfer2(write_list)
        self.gains[channel] = value
        if immediate is True:
            self.load_dac()

    def write_gain_factor(self, channel, value, immediate=True):
        m = int((value * 2**16) - 1)
        self.write_gain_int(channel, m, immediate=immediate)

    def get_gain_factor(self, channel):
        gain = (self.gains[channel] + 1) / 2**16
        return gain

    def get_gain_int(self, channel):
        return self.gains[channel]

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
        write_list = [int(x + a), d[1], d[0]]
        self.spi.xfer2(write_list)

    def read_register(self, reg):
        """
        Read a register from the AD5370 using the function register.
        It seems this is not available for the eval board, as sdo
        signal is not routed to the header.

        :param reg: Register code. Consult data sheet for details
        :return:
        """
        d = struct.pack('<H', reg)
        write_list = [0b00000101, d[1], d[0]]
        self.spi.xfer2(write_list)
        res = self.spi.xfer2([0, 0, 0])
        return res

    def read_control_register(self):
        res = self.read_register(0b1000_0000_1000_0000)
        return res



if __name__ == '__main__':
    ac = AD5370_control()