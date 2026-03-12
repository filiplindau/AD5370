"""
Modern style pytango device for AD5370. Also exposing gain and offset values for the DACs.

:author: Filip Lindau
:created: 2026-02-27
"""
import sys
import PyTango
from PyTango.server import Device, DeviceMeta, device_property, attribute, command
import AD5370_rpi_control as ad
import threading
import logging
import time
import numpy as np
import queue

logging.basicConfig(format='%(asctime)s - %(module)s.   %(funcName)s - %(levelname)s - %(message)s', level=logging.INFO)


class Command:
    def __init__(self, command, data=None):
        self.command = command
        self.data = data

# ==================================================================
#   AD5370_dac_DS Class Description:
#
#         Control of a AD5370 DAC
#
# ==================================================================
#     Device States Description:
#
#   DevState.ON :       Connected to Halcyon driver
#   DevState.OFF :      Disconnected from Halcyon
#   DevState.FAULT :    Error detected
#   DevState.UNKNOWN :  Communication problem
#   DevState.INIT :     Initializing Halcyon driver.
# ==================================================================


class AD5370DacDS(Device):
    __metaclass__ = DeviceMeta

    spi_port = device_property(dtype=int,
                               doc="SPI port on the raspberry the AD5370_dac_ is connected to (should be 0)",
                               default_value=0)

    spi_device = device_property(dtype=int,
                                 doc="SPI device (CS) on the raspberry the AD5370_dac_ is using",
                                 default_value=0)

    voltage_reference = device_property(dtype=float,
                                        doc="Voltage reference for the AD5370 (determines voltage span and precision)",
                                        default_value=5.0)

    # --------- Device attributes --------------------------
    # Additional attributes will be created dynamically.
    #
    apply_immediate = attribute(label="apply immediate",
                                dtype=bool,
                                access=PyTango.AttrWriteType.READ_WRITE,
                                unit="",
                                fget="get_apply_immediate",
                                fset="set_apply_immediate",
                                fisallowed="is_apply_immediate_allowed",
                                doc="Determines if written dac values are applied immediately or if APPLY command is needed",
                                memorized=True,
                                )

    offset_code = attribute(label="offset code",
                                dtype=int,
                                access=PyTango.AttrWriteType.READ_WRITE,
                                unit="",
                            max_value=16383,
                            min_value=0,
                                fget="get_offset_code",
                                fset="set_offset_code",
                                fisallowed="is_offset_code_allowed",
                                doc="DAC offset value that is common to all channels",
                                memorized=True,
                                )

    v_min = attribute(label="v_min",
                      dtype=(float, ),
                      max_dim_x=40,
                      access=PyTango.AttrWriteType.READ,
                      unit="V",
                      fget="get_vmin",
                      fisallowed="is_vmin_allowed",
                      doc="Minimum voltage with current settings",
                      )

    v_max = attribute(label="v_max",
                      dtype=(float, ),
                      max_dim_x=40,
                      access=PyTango.AttrWriteType.READ,
                      unit="V",
                      fget="get_vmax",
                      fisallowed="is_vmax_allowed",
                      doc="Maximum voltage with current settings",
                      )


    # ------------------------------------------------------------------
    #     Device constructor
    # ------------------------------------------------------------------
    def __init__(self, cl, name):

        self.stream_lock = None
        self.attr_lock = None
        self.state_thread = None
        self.command_queue = None
        self.state_handler_dict = None
        self.stop_state_thread_flag = None
        self.AD5370_dac_device: ad.AD5370_control = None
        self._apply_immediate = False

        Device.__init__(self, cl, name)

    # ------------------------------------------------------------------
    #     Device destructor
    # ------------------------------------------------------------------
    def delete_device(self):
        with self.stream_lock:
            self.info_stream(''.join(("[Device delete_device method] for device", self.get_name())))
        self.stop_thread()

    # ------------------------------------------------------------------
    #     Device initialization
    # ------------------------------------------------------------------
    def init_device(self):
        self.stream_lock = threading.Lock()
        with self.stream_lock:
            self.info_stream(''.join(("In ", self.get_name(), "::init_device()")))
        self.set_state(PyTango.DevState.UNKNOWN)
        self.get_device_properties(self.get_device_class())

        # Try stopping the stateThread if it was started before. Will fail if this
        # is the initial start.
        try:
            self.stop_thread()

        except Exception as e:
            pass

        self._apply_immediate = False

        self.attr_lock = threading.Lock()
        self.state_thread = threading.Thread()
        threading.Thread.__init__(self.state_thread, target=self.state_handler_dispatcher)

        self.command_queue = queue.Queue(100)

        try:
            # Add channel attributes if not already defined
            attrs = self.get_device_attr()
            for ch in range(40):
                try:
                    attrs.get_attr_by_name(f"channel{str(ch)}")
                    self.info_stream(f"channel{str(ch)} already defined.")
                except PyTango.DevFailed:
                    # The attribute was not defined, so add it
                    self.info_stream(f"channel{str(ch)} not defined, adding attributes")
                    attr_name = f"channel{str(ch)}"
                    attr = attribute(
                        name=attr_name,
                        dtype=float,
                        access=PyTango.READ_WRITE,
                        fget=self.read_channel,
                        fset=self.write_channel,
                        fisallowed=self.is_channel_allowed,
                        doc="DAC output voltage for channel",
                        unit="V",
                        memorized=True)
                    self.add_attribute(attr)

                    attr_name = f"channel{str(ch)}_raw"
                    attr = attribute(
                        name=attr_name,
                        dtype=int,
                        access=PyTango.READ,
                        fget=self.read_channel_raw,
                        fisallowed=self.is_channel_allowed,
                        doc="DAC raw output for channel",
                        unit="counts",
                        display_level=PyTango.DispLevel.EXPERT)
                    self.add_attribute(attr)

                    attr_name = f"gain{str(ch)}"
                    attr = attribute(
                        name=attr_name,
                        dtype=float,
                        access=PyTango.READ_WRITE,
                        fget=self.read_gain,
                        fset=self.write_gain,
                        fisallowed=self.is_gain_allowed,
                        doc="DAC gain for channel",
                        unit="",
                        memorized=True)
                    self.add_attribute(attr)

                    attr_name = f"gain{str(ch)}_raw"
                    attr = attribute(
                        name=attr_name,
                        dtype=int,
                        access=PyTango.READ,
                        fget=self.read_gain_raw,
                        fisallowed=self.is_gain_allowed,
                        doc="DAC raw gain for channel",
                        unit="counts",
                        display_level=PyTango.DispLevel.EXPERT)
                    self.add_attribute(attr)

                    attr_name = f"offset{str(ch)}"
                    attr = attribute(
                        name=attr_name,
                        dtype=float,
                        access=PyTango.READ_WRITE,
                        fget=self.read_offset,
                        fset=self.write_offset,
                        fisallowed=self.is_offset_allowed,
                        doc="DAC offset for channel",
                        unit="V",
                        memorized=True)
                    self.add_attribute(attr)

                    attr_name = f"offset{str(ch)}_raw"
                    attr = attribute(
                        name=attr_name,
                        dtype=int,
                        access=PyTango.READ,
                        fget=self.read_offset_raw,
                        fisallowed=self.is_offset_allowed,
                        doc="DAC raw offset for channel",
                        unit="counts",
                        display_level=PyTango.DispLevel.EXPERT)
                    self.add_attribute(attr)

        except Exception as ex:
            with self.stream_lock:
                self.error_stream('Error when initializing device')
                self.error_stream(str(ex))

        self.state_handler_dict = {PyTango.DevState.ON: self.on_handler,
                                   PyTango.DevState.MOVING: self.on_handler,
                                   PyTango.DevState.ALARM: self.on_handler,
                                   PyTango.DevState.FAULT: self.fault_handler,
                                   PyTango.DevState.INIT: self.init_handler,
                                   PyTango.DevState.UNKNOWN: self.unknown_handler,
                                   PyTango.DevState.OFF: self.off_handler}

        self.stop_state_thread_flag = False

        self.state_thread.start()

    def state_handler_dispatcher(self):
        """Handles switch of states in the state machine thread.
        Each state handled method should exit by setting the next state,
        going back to this method. The previous state is also included when
        calling the next state handler method.
        The thread is stopped by setting the stopStateThreadFlag.
        """
        prev_state = self.get_state()
        while self.stop_state_thread_flag is False:
            try:
                self.state_handler_dict[self.get_state()](prev_state)
                prev_state = self.get_state()
            except KeyError:
                self.state_handler_dict[PyTango.DevState.UNKNOWN](prev_state)
                prev_state = self.get_state()

    def stop_thread(self):
        """Stops the state handler thread by setting the stopStateThreadFlag
        """
        self.stop_state_thread_flag = True
        self.state_thread.join(3)
        self.AD5370_dac_device.close()

    def unknown_handler(self, prev_state):
        """Handles the UNKNOWN state, before communication with the hardware devices
        has been established. Here all devices are initialized.
        """
        with self.stream_lock:
            self.info_stream('Entering unknown_handler')
        connection_timeout = 1.0
        self.set_status('Connecting to AD5370_dac through spi bus')
        while self.stop_state_thread_flag is False:
            try:
                self.AD5370_dac_device.close()
            except:
                pass
            try:
                with self.stream_lock:
                    self.info_stream(''.join(('Opening AD5370_dac device on port ', str(self.spi_port))))
                self.AD5370_dac_device = ad.AD5370_control(self.spi_port, self.spi_device, self.voltage_reference)
            except Exception as ex:
                with self.stream_lock:
                    self.error_stream(''.join(('Could not connect to AD5370_dac on address ', str(self.spi_port))))
                with self.stream_lock:
                    self.error_stream(str(ex))
                self.set_status('Could not connect to AD5370_dac')
                continue
            self.set_state(PyTango.DevState.INIT)
            break

    def init_handler(self, prev_state):
        """Handles the INIT state. Query AD5370 device to see if it is alive.
        """
        with self.stream_lock:
            self.info_stream('Entering initHandler')
        wait_time = 0.1
        self.set_status('Initializing device')
        retries = 0
        max_tries = 5

        while self.stop_state_thread_flag is False:
            retries += 1
            if retries > max_tries:
                self.set_state(PyTango.DevState.UNKNOWN)
                break

            # Set channels to memorized values
            attrs = self.get_device_attr()
            self.AD5370_dac_device.reset()
            attr = attrs.get_attr_by_name(f"offset_code")
            self.AD5370_dac_device.write_offset_code(attr.get_write_value(), False)
            self.AD5370_dac_device.clear(True)
            for ch in range(40):
                try:
                    attr = attrs.get_attr_by_name(f"channel{str(ch)}")
                    self.AD5370_dac_device.write_value_volt(ch, attr.get_write_value(), False)
                    time.sleep(0.05)
                    attr = attrs.get_attr_by_name(f"gain{str(ch)}")
                    self.AD5370_dac_device.write_gain_factor(ch, attr.get_write_value(), False)
                    time.sleep(0.05)
                    attr = attrs.get_attr_by_name(f"offset{str(ch)}")
                    self.AD5370_dac_device.write_offset_volt(ch, attr.get_write_value(), False)
                    time.sleep(0.05)
                    self.set_status(f"Initializing device\n... ch{ch+1}/40")
                except ValueError as e:
                    self.error_stream(f"Error setting channel {ch}: {e}")
                    break
            self.AD5370_dac_device.load_dac()

            self.check_commands(block_time=wait_time)
            if self.command_queue.empty:
                self.set_state(PyTango.DevState.OFF)
                break
            else:
                retries -= 1


    def on_handler(self, prev_state):
        """Handles the ON state. Connected to the AD5370_dac.
        Loops checking commands. There is nothing to read from the device.
        """
        with self.stream_lock:
            self.info_stream('Entering onHandler')
        handled_states = [PyTango.DevState.ON, PyTango.DevState.ALARM, PyTango.DevState.MOVING]
        wait_time = 0.025
        self.set_status('On')
        while self.stop_state_thread_flag is False:
            # self.info_stream('onhandler loop')
            with self.attr_lock:
                state = self.get_state()
            if state not in handled_states:
                self.info_stream(''.join(('Exit onhandler, state=', str(state), ', stopStateThreadFlag=', str(self.stop_state_thread_flag))))
                break
            self.check_commands(block_time=wait_time)

    def fault_handler(self, prev_state):
        """Handles the FAULT state. A problem has been detected.
        """
        with self.stream_lock:
            self.info_stream('Entering faultHandler')
        handled_states = [PyTango.DevState.FAULT]
        wait_time = 0.1
        retries = 0
        max_tries = 5

        while self.stop_state_thread_flag is False:
            if self.get_state() not in handled_states:
                break
            # Test AD5370_dac_:
            try:
                with self.stream_lock:
                    self.debug_stream('Resetting...')
                self.AD5370_dac_device.reset()

                for channel in range(40):
                    with self.attr_lock:
                        value = self.AD5370_dac_device.get_output_volt(channel)
                        self.AD5370_dac_device.write_value_volt(channel, value, self._apply_immediate)
                    s = ''.join(('Channel ', str(channel), ': ', str(value)))
                    self.debug_stream(s)

            except Exception as e:
                self.set_state(PyTango.DevState.UNKNOWN)

            if self.get_state() == PyTango.DevState.FAULT:
                retries += 1
                if retries > max_tries:
                    self.set_state(PyTango.DevState.UNKNOWN)
            self.check_commands(block_time=wait_time)

    def off_handler(self, prev_state):
        """Handles the OFF state. Does nothing, just goes back to ON.
        """
        with self.stream_lock:
            self.info_stream('Entering offHandler')
        handled_states = [PyTango.DevState.OFF]
        wait_time = 0.025
        self.set_status('Off')
        while self.stop_state_thread_flag is False:
            with self.attr_lock:
                state = self.get_state()
            if state not in handled_states:
                self.info_stream(f"Exit off_handler, state={str(state)}, stopStateThreadFlag={str(self.stop_state_thread_flag)}")
                break
            self.check_commands(block_time=wait_time)

    def check_commands(self, block_time=0):
        """Checks the commandQueue for new commands. Must be called regularly.
        If the queue is empty the method exits immediately.
        """
#         with self.streamLock:
#             self.debug_stream('Entering checkCommands')
        try:
            if block_time == 0:
                cmd = self.command_queue.get(block=False)
            else:
                cmd = self.command_queue.get(block=True, timeout=block_time)

            with self.stream_lock:
                self.info_stream(''.join(('Command ', str(cmd.command), ': ', str(cmd.data))))

            if cmd.command == 'off':
                if self.get_state() not in [PyTango.DevState.INIT, PyTango.DevState.UNKNOWN]:
                    with self.attr_lock:
                        self.AD5370_dac_device.clear(True)
                    self.set_state(PyTango.DevState.OFF)

            elif cmd.command == 'init':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    self.set_state(PyTango.DevState.UNKNOWN)

            elif cmd.command == 'alarm':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    self.set_state(PyTango.DevState.ALARM)

            elif cmd.command == 'on':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    with self.attr_lock:
                        self.AD5370_dac_device.clear(False)
                    self.set_state(PyTango.DevState.ON)

            elif cmd.command == 'reset':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    with self.attr_lock:
                        self.AD5370_dac_device.reset()

            elif cmd.command == 'apply':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    with self.attr_lock:
                        self.AD5370_dac_device.load_dac()

            elif cmd.command == 'write_apply_immediate':
                if self.get_state() not in []:
                    # If we change to apply immediate == True, apply all values now:
                    if cmd.data is True and self._apply_immediate is False:
                        self.info_stream('From write_apply_immediate: Adding apply command to queue')
                        cmd_msg = Command('apply')
                        with self.attr_lock:
                            self.command_queue.put(cmd_msg)
                    self._apply_immediate = cmd.data

            elif cmd.command == 'write_channel':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    with self.attr_lock:
                        self.info_stream(
                            'From write_channel: Setting channel' + str(cmd.data[0]) + " to " + str(cmd.data[1]) + " V")
                        try:
                            self.AD5370_dac_device.write_value_volt(cmd.data[0], cmd.data[1], self._apply_immediate)
                        except ValueError as e:
                            self.error_stream(f"Error writing channel {cmd.data[0]} to {cmd.data[1]}: \n{e}")

            elif cmd.command == 'write_gain':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    with self.attr_lock:
                        self.info_stream(
                            'From write_gain: Setting gain' + str(cmd.data[0]) + " to " + str(cmd.data[1]) + " V")
                        try:
                            self.AD5370_dac_device.write_gain_factor(cmd.data[0], cmd.data[1], self._apply_immediate)
                        except ValueError as e:
                            self.error_stream(f"Error writing channel {cmd.data[0]} to {cmd.data[1]}: \n{e}")

            elif cmd.command == 'write_offset':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    with self.attr_lock:
                        self.info_stream(
                            'From write_offset: Setting offset' + str(cmd.data[0]) + " to " + str(cmd.data[1]) + " V")
                        try:
                            self.AD5370_dac_device.write_offset_volt(cmd.data[0], cmd.data[1], self._apply_immediate)
                        except ValueError as e:
                            self.error_stream(f"Error writing channel {cmd.data[0]} to {cmd.data[1]}: \n{e}")

            elif cmd.command == 'write_offset_code':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    with self.attr_lock:
                        self.info_stream(f"From write_offset_code: Setting offset_code to {str(cmd.data)}")
                        try:
                            self.AD5370_dac_device.write_offset_code(cmd.data, self._apply_immediate)
                        except ValueError as e:
                            self.error_stream(f"Error writing offset_code {cmd.data}: \n{e}")

        except queue.Empty:
            # with self.streamLock:
            # self.debug_stream('checkCommands: queue empty')
            pass

# ------------------------------------------------------------------
#     Always excuted hook method
# ------------------------------------------------------------------
    def always_executed_hook(self):
        pass

# ------------------------------------------------------------------
#     Channel attribute
# ------------------------------------------------------------------
    def read_channel(self, attr):
        with self.stream_lock:
            self.info_stream(''.join(('Reading channel for ', attr.get_name())))
        ch = int(attr.get_name().rsplit('channel')[1])
        with self.attr_lock:
            value = self.AD5370_dac_device.get_output_volt(ch)
            if value is None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                value = 0.0
        return value

    def write_channel(self, attr):
        self.info_stream(''.join(('Writing channel for ', attr.get_name())))
        ch = int(attr.get_name().rsplit('channel')[1])
        with self.attr_lock:
            data = attr.get_write_value()
            self.info_stream(''.join(('Setting channel ', str(ch), ' to ', str(data))))
            cmd_msg = Command('write_channel', [ch, data])
            self.command_queue.put(cmd_msg)

    def read_channel_raw(self, attr):
        with self.stream_lock:
            self.info_stream(''.join(('Reading raw channel for ', attr.get_name())))
        ch = int(attr.get_name().rsplit('channel')[1][:-4])
        with self.attr_lock:
            value = self.AD5370_dac_device.get_output_int(ch)
            if value is None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                value = 0
        return value

    def is_channel_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

    # ------------------------------------------------------------------
    #     Gain attribute
    # ------------------------------------------------------------------
    def read_gain(self, attr):
        with self.stream_lock:
            self.info_stream(''.join(('Reading gain for ', attr.get_name())))
        ch = int(attr.get_name().rsplit('gain')[1])
        with self.attr_lock:
            value = self.AD5370_dac_device.get_gain_factor(ch)
            if value is None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                value = 1.0
        return value

    def write_gain(self, attr):
        self.info_stream(''.join(('Writing gain for ', attr.get_name())))
        ch = int(attr.get_name().rsplit('gain')[1])
        with self.attr_lock:
            data = attr.get_write_value()
            self.info_stream(''.join(('Setting gain ', str(ch), ' to ', str(data))))
            cmd_msg = Command('write_gain', [ch, data])
            self.command_queue.put(cmd_msg)

    def read_gain_raw(self, attr):
        with self.stream_lock:
            self.info_stream(''.join(('Reading raw gain for ', attr.get_name())))
        ch = int(attr.get_name().rsplit('gain')[1][:-4])
        with self.attr_lock:
            value = self.AD5370_dac_device.get_gain_int(ch)
            if value is None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                value = 65535
        return value

    def is_gain_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

    # ------------------------------------------------------------------
    #     Offset attribute
    # ------------------------------------------------------------------
    def read_offset(self, attr):
        with self.stream_lock:
            self.info_stream(''.join(('Reading offset for ', attr.get_name())))
        ch = int(attr.get_name().rsplit('offset')[1])
        with self.attr_lock:
            value = self.AD5370_dac_device.get_offset_volt(ch)
            if value is None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                value = 1.0
        return value

    def write_offset(self, attr):
        self.info_stream(''.join(('Writing offset for ', attr.get_name())))
        ch = int(attr.get_name().rsplit('offset')[1])
        with self.attr_lock:
            data = attr.get_write_value()
            self.info_stream(''.join(('Setting offset ', str(ch), ' to ', str(data))))
            cmd_msg = Command('write_offset', [ch, data])
            self.command_queue.put(cmd_msg)

    def read_offset_raw(self, attr):
        with self.stream_lock:
            self.info_stream(''.join(('Reading raw offset for ', attr.get_name())))
        ch = int(attr.get_name().rsplit('offset')[1][:-4])
        with self.attr_lock:
            value = self.AD5370_dac_device.get_offset_dac(ch)
            if value is None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                value = 0
        return value

    def is_offset_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

# ------------------------------------------------------------------
#     apply_immediate attribute
# ------------------------------------------------------------------
    def get_apply_immediate(self):
        with self.stream_lock:
            self.info_stream(''.join(('Reading apply immediate')))
        with self.attr_lock:
            attr_read = self._apply_immediate
            if attr_read is None:
                q = PyTango.AttrQuality.ATTR_INVALID
                attr_read = False
            else:
                q = PyTango.AttrQuality.ATTR_VALID
        return attr_read, time.time(), q

    def set_apply_immediate(self, value):
        self.info_stream(''.join(('Writing apply immediate')))
        with self.attr_lock:
            data = value
            self.info_stream(''.join(('Setting apply immediate to ', str(data))))
            cmd_msg = Command('write_apply_immediate', data)
            self.command_queue.put(cmd_msg)

    def is_apply_immediate_allowed(self, req_type):
        if self.get_state() in []:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

    # ------------------------------------------------------------------
    #     offset_code attribute
    # ------------------------------------------------------------------
    def get_offset_code(self):
        with self.stream_lock:
            self.info_stream(''.join(('Reading offset_code')))
        with self.attr_lock:
            attr_read = self.AD5370_dac_device.get_offset_code_int()
            if attr_read is None:
                q = PyTango.AttrQuality.ATTR_INVALID
                attr_read = False
            else:
                q = PyTango.AttrQuality.ATTR_VALID
        return attr_read, time.time(), q

    def set_offset_code(self, value):
        self.info_stream(''.join(('Writing offset_code')))
        with self.attr_lock:
            data = value
            self.info_stream(''.join(('Setting offset_code to ', str(data))))
            cmd_msg = Command('write_offset_code', data)
            self.command_queue.put(cmd_msg)

    def is_offset_code_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

    # ------------------------------------------------------------------
    #     vmin attribute
    # ------------------------------------------------------------------
    def get_vmin(self):
        with self.attr_lock:
            attr_read = np.array(self.AD5370_dac_device.V_min)
            self.info_stream(f"Reading vmin: {attr_read}")
            if attr_read is None:
                q = PyTango.AttrQuality.ATTR_INVALID
                attr_read = False
            else:
                q = PyTango.AttrQuality.ATTR_VALID
        return attr_read, time.time(), q

    def is_vmin_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

    # ------------------------------------------------------------------
    #     vmax attribute
    # ------------------------------------------------------------------
    def get_vmax(self):
        with self.attr_lock:
            attr_read = np.array(self.AD5370_dac_device.V_max)
            self.info_stream(f"Reading vmax: {attr_read}")
            if attr_read is None:
                q = PyTango.AttrQuality.ATTR_INVALID
                attr_read = False
            else:
                q = PyTango.AttrQuality.ATTR_VALID
        return attr_read, time.time(), q

    def is_vmax_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True
# ==================================================================
#
#     AD5370DacDS command methods
#
# ==================================================================

# ------------------------------------------------------------------
#     On command:
#
#     Description: Start AD5370
#
# ------------------------------------------------------------------
    @command(dtype_in=None, doc_in="Turn ON dac outputs")
    def On(self):
        with self.stream_lock:
            self.info_stream(''.join(("In ", self.get_name(), "::On")))
        cmd_msg = Command('on')
        self.command_queue.put(cmd_msg)

    def is_On_allowed(self):
        if self.get_state() in [PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

    @command(dtype_in=None, doc_in="Turn OFF dac outputs (set to ground potential)")
    def Off(self):
        with self.stream_lock:
            self.info_stream(''.join(("In ", self.get_name(), "::Off")))
        cmd_msg = Command('off')
        self.command_queue.put(cmd_msg)

    @command(dtype_in=None, doc_in="Apply channel values to dac outputs")
    def Apply(self):
        with self.stream_lock:
            self.info_stream(''.join(("In ", self.get_name(), "::Apply")))
        cmd_msg = Command('apply')
        self.command_queue.put(cmd_msg)

        # ---- Apply command State Machine -----------------

    def is_Apply_allowed(self):
        if self.get_state() in [PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

    @command(dtype_in=None, doc_in="Reset AD5370")
    def Reset(self):
        with self.stream_lock:
            self.info_stream(''.join(("In ", self.get_name(), "::Reset")))
        cmd_msg = Command('reset')
        self.command_queue.put(cmd_msg)

        # ---- Reset command State Machine -----------------

    def is_Reset_allowed(self):
        if self.get_state() in [PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True





# ==================================================================
#
#     AD5370DacDS class main method
#
# ==================================================================
if __name__ == '__main__':
    PyTango.server.server_run((AD5370DacDS,))

