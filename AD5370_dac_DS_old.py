"""Created on 08 apr 2016

@author: Filip Lindau
"""
import sys
import PyTango
import AD5370_rpi_control as ad
import threading
import logging
import time
import numpy as np
import Queue

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


class AD5370DacDS(PyTango.Device_4Impl):

    # --------- Add your global variables here --------------------------

    # ------------------------------------------------------------------
    #     Device constructor
    # ------------------------------------------------------------------
    def __init__(self, cl, name):
        PyTango.Device_4Impl.__init__(self, cl, name)

        self.stream_lock = None
        self.attr_lock = None
        self.apply_immediate = None
        self.state_thread = None
        self.command_queue = None
        self.state_handler_dict = None
        self.stop_state_thread_flag = None
        self.AD5370_dac_device = None

        AD5370DacDS.init_device(self)

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

        except Exception, e:
            pass

        self.apply_immediate = False

        self.attr_lock = threading.Lock()
        self.state_thread = threading.Thread()
        threading.Thread.__init__(self.state_thread, target=self.state_handler_dispatcher)

        self.command_queue = Queue.Queue(100)

        try:
            # Add channel attributes if not already defined
            attrs = self.get_device_attr()
            for ch in range(40):
                try:
                    attrs.get_attr_by_name(''.join(('channel', str(ch))))
                    self.info_stream(''.join(('Channel ', str(ch), ' already defined.')))
                except PyTango.DevFailed:
                    # The attribute was not defined, so add it
                    self.info_stream(''.join(('Channel ', str(ch), ' not defined, adding attribute.')))
                    attr_info = [[PyTango.DevDouble, PyTango.SCALAR, PyTango.READ_WRITE],
                                 {
                                     'description': "DAC output for channel",
                                     'unit': 'V',
                                     'Memorized': "true",
                                 }]
                    attr_name = ''.join(('channel', str(ch)))
                    attr_data = PyTango.AttrData(attr_name, self.get_name(), attr_info)
                    self.add_attribute(attr_data, r_meth=self.read_channel, w_meth=self.write_channel,
                                       is_allo_meth=self.is_channel_allowed)

        except Exception, ex:
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
                self.AD5370_dac_device = ad.AD5370_control(self.spi_port, self.spi_device)
            except Exception, ex:
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

            self.check_commands(block_time=wait_time)
            if self.command_queue.empty:
                self.set_state(PyTango.DevState.ON)
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
                        value = self.AD5370_dac_device.voltages[channel]
                        self.AD5370_dac_device.write_value_volt(channel, value, self.apply_immediate)
                    s = ''.join(('Channel ', str(channel), ': ', str(value)))
                    self.debug_stream(s)

            except Exception, e:
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
        self.set_state(PyTango.DevState.ON)

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
                    self.set_state(PyTango.DevState.OFF)

            elif cmd.command == 'init':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    self.set_state(PyTango.DevState.UNKNOWN)

            elif cmd.command == 'alarm':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    self.set_state(PyTango.DevState.ALARM)

            elif cmd.command == 'on':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    self.set_state(PyTango.DevState.ON)

            elif cmd.command == 'reset':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    with self.attr_lock:
                        self.AD5370_dac_device.reset()

            elif cmd.command == 'apply':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    with self.attr_lock:
                        for ch in range(40):
                            self.AD5370_dac_device.write_value_volt(ch, self.AD5370_dac_device.voltages[ch], False)
                        self.AD5370_dac_device.load_dac()

            elif cmd.command == 'write_apply_immediate':
                if self.get_state() not in []:
                    # If we change to apply immediate == True, apply all values now:
                    if cmd.data is True and self.apply_immediate is False:
                        self.info_stream('From write_apply_immediate: Adding apply command to queue')
                        cmd_msg = Command('apply')
                        with self.attr_lock:
                            self.command_queue.put(cmd_msg)
                    self.apply_immediate = cmd.data

            elif cmd.command == 'write_channel':
                if self.get_state() not in [PyTango.DevState.UNKNOWN]:
                    with self.attr_lock:
                        self.info_stream(
                            'From write_channel: Setting channel' + str(cmd.data[0]) + " to " + str(cmd.data[1]) + " V")
                        self.AD5370_dac_device.write_value_volt(cmd.data[0], cmd.data[1], self.apply_immediate)

        except Queue.Empty:
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
            attr_read = self.AD5370_dac_device.voltages[ch]
            if attr_read is None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                attr_read = 0.0
            attr.set_value(attr_read)

    def write_channel(self, attr):
        self.info_stream(''.join(('Writing channel for ', attr.get_name())))
        ch = int(attr.get_name().rsplit('channel')[1])
        with self.attr_lock:
            data = attr.get_write_value()
            self.info_stream(''.join(('Setting channel ', str(ch), ' to ', str(data))))
            cmd_msg = Command('write_channel', [ch, data])
            self.command_queue.put(cmd_msg)

    def is_channel_allowed(self, req_type):
        if self.get_state() in [PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

# ------------------------------------------------------------------
#     apply_immediate attribute
# ------------------------------------------------------------------
    def read_apply_immediate(self, attr):
        with self.stream_lock:
            self.info_stream(''.join(('Reading apply immediate')))
        with self.attr_lock:
            attr_read = self.apply_immediate
            if attr_read is None:
                attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
                attr_read = False
            attr.set_value(attr_read)

    def write_apply_immediate(self, attr):
        self.info_stream(''.join(('Writing apply immediate')))
        with self.attr_lock:
            data = attr.get_write_value()
            self.info_stream(''.join(('Setting apply immediate to ', str(data))))
            cmd_msg = Command('write_apply_immediate', data)
            self.command_queue.put(cmd_msg)

    def is_apply_immediate_allowed(self, req_type):
        if self.get_state() in []:
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
    def On(self):
        with self.stream_lock:
            self.info_stream(''.join(("In ", self.get_name(), "::On")))
        cmd_msg = Command('on')
        self.command_queue.put(cmd_msg)

# ---- On command State Machine -----------------
    def is_On_allowed(self):
        if self.get_state() in [PyTango.DevState.UNKNOWN]:
            #     End of Generated Code
            #     Re-Start of Generated Code
            return False
        return True

# ------------------------------------------------------------------
#     Apply command:
#
#     Description: Apply written dac values to outputs
#
# ------------------------------------------------------------------
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

# ------------------------------------------------------------------
#     Reset command:
#
#     Description: Reset AD5370
#
# ------------------------------------------------------------------
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
#     AD5370DacDSClass class definition
#
# ==================================================================
class AD5370DacDSClass(PyTango.DeviceClass):

    #     Class Properties
    class_property_list = {
        }

    #     Device Properties
    device_property_list = {
        'spi_port':
            [PyTango.DevLong,
             "SPI port on the raspberry the AD5370_dac_ is connected to (should be 0)",
             [0]],
        'spi_device':
            [PyTango.DevLong,
             "SPI device (CS) on the raspberry the AD5370_dac_ is using",
             [0]],
        'voltage_reference':
            [PyTango.DevDouble,
             "Voltage reference for the AD5370 (determines voltage span and precision)",
             [5.0]],
    }

    #     Command definitions
    cmd_list = {
        'On':
            [[PyTango.DevVoid, ""],
             [PyTango.DevVoid, ""]],
        'Apply':
            [[PyTango.DevVoid, ""],
             [PyTango.DevVoid, ""]],
        'Reset':
            [[PyTango.DevVoid, ""],
             [PyTango.DevVoid, ""]],
        }

    #     Attribute definitions
    attr_list = {
        'apply_immediate':
            [[PyTango.DevBoolean,
             PyTango.SCALAR,
             PyTango.READ_WRITE],
             {
                'description': "Determines if written dac values are applied immediately or if APPLY command is needed",
                'unit': '',
                'Memorized': "true",
             }],


        }

# ------------------------------------------------------------------
#     AD5370DacDSClass Constructor
# ------------------------------------------------------------------
    def __init__(self, name):
        PyTango.DeviceClass.__init__(self, name)
        self.set_type(name);
        print "In AD5370DacDSClass constructor"

# ==================================================================
#
#     AD5370DacDS class main method
#
# ==================================================================
if __name__ == '__main__':
    try:
        py = PyTango.Util(sys.argv)
        py.add_class(AD5370DacDSClass, AD5370DacDS, 'AD5370DacDS')

        U = PyTango.Util.instance()
        U.server_init()
        U.server_run()

    except PyTango.DevFailed, e:
        print '-------> Received a DevFailed exception:', e
    except Exception, e:
        print '-------> An unforeseen exception occured....', e
