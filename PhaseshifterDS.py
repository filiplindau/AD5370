import time
import PyTango as pt
from PyTango.server import Device, DeviceMeta
from PyTango.server import attribute
from PyTango.server import device_property


class PhaseshifterDS(Device):
    __metaclass__ = DeviceMeta

    phase = attribute(label="Phase",
                      dtype=float,
                      access=pt.AttrWriteType.READ_WRITE,
                      unit="deg",
                      format="3.2f",
                      min_value=0.0,
                      max_value=720.0,
                      fget="get_phase",
                      fset="set_phase",
                      doc="RF phase",
                      memorized=True,
                      hw_memorized=True)

    dac_ds_name = device_property(dtype=str,
                                  doc="Name of the underlying AD5370 DAC device server")
    phase_volt_cal = device_property(dtype=float,
                                     doc="Calibration factor in degrees/volt",
                                     default_value=36.0)
    dac_channel = device_property(dtype=int,
                                  doc="Channel connected to the phase shifter",
                                  default_value=39)

    def init_device(self):
        self.debug_stream("In init_device:")
        Device.init_device(self)
        self.phase_val = 0.0
        self.dac_dev = pt.DeviceProxy(self.dac_ds_name)
        self.set_state(pt.DevState.ON)

    def get_phase(self):
        self.debug_stream("In get_phase:")
        return self.phase_val, time.time(), pt.AttrQuality.ATTR_VALID

    def set_phase(self, new_phase):
        self.debug_stream("In set_phase: New phase " + str(new_phase))
        self.phase_val = new_phase
        self.dac_dev.write_attribute("".join(("channel", str(self.dac_channel))), new_phase / self.phase_volt_cal)


if __name__ == "__main__":
    pt.server.server_run((PhaseshifterDS,))
