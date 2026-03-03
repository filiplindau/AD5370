import time
import PyTango as pt
from PyTango.server import Device, DeviceMeta
from PyTango.server import attribute
from PyTango.server import device_property
from scipy.interpolate import interp1d
import numpy as np

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
    phase_calibration_data = device_property(dtype=str,
                                             doc="Calibration phase data vector")
    voltage_calibration_data = device_property(dtype=str,
                                               doc="Calibration voltage data vector")

    def init_device(self):
        self.debug_stream("In init_device:")
        Device.init_device(self)
        try:
            self.u = np.array(self.voltage_calibration_data.split(",")).astype(np.double)
            self.debug_stream("Voltage calibration data: {0}, length {1}".format(self.u, len(self.u)))
            self.ph = np.array(self.phase_calibration_data.split(",")).astype(np.double)
            self.debug_stream("Phase calibration data: {0}, length {1}".format(self.ph, len(self.ph)))
            if len(self.u) != len(self.ph):
                raise ValueError("Wrong dimension")
            self.condition_calibration_data()
            self.ph_interp = interp1d(self.ph, self.u)
        except:
            self.error_stream("Wrong dimension of calibration data, using stock")
            self.u = np.array(
                [0, 0.25, 0.5, 0.75, 1., 1.25, 1.5, 1.75, 2., 2.25, 2.5, 2.75, 3., 3.25, 3.5, 3.75, 4., 4.25,
                 4.5, 4.75, 5., 5.25, 5.5, 5.75, 6., 6.25, 6.5, 6.75, 7., 7.25, 7.5, 7.75, 8., 8.25, 8.5,
                 8.75, 9., 9.25, 9.5, 9.75, 10.])
            self.ph = np.array([0., 7.5, 15.7, 23.5, 32.5, 41.5, 52., 63.4, 76.6, 91.5, 109., 129.1,
                                152.6, 179.3, 208.8, 239.9, 271.2, 300.9, 328.8, 354., 376.9, 397., 414.2,
                                429.5, 442.4, 453.4, 462.7, 470.6, 477.4, 483.2, 488.3, 492.5, 496.2, 499.9,
                                502.1, 504.5, 506.6, 508.4, 510., 511.4, 512.7])
            self.ph_interp = interp1d(self.ph, self.u)
        self.phase_val = 0.0
        self.dac_dev = pt.DeviceProxy(self.dac_ds_name)
        self.set_state(pt.DevState.ON)

    def condition_calibration_data(self):
        ph0 = self.ph[0]
        dph = self.ph - ph0
        add_ph = [0.0]
        for i in range(len(dph) - 1):
            new_ph = add_ph[-1]
            if abs(dph[i + 1] - dph[i]) > 180.0:
                new_ph += 360.0
            add_ph.append(new_ph)
        add_ph = np.array(add_ph)
        c_ph = add_ph + dph
        self.ph = c_ph

    def get_phase(self):
        self.debug_stream("In get_phase:")
        return self.phase_val, time.time(), pt.AttrQuality.ATTR_VALID

    def set_phase(self, new_phase):
        self.debug_stream("In set_phase: New phase " + str(new_phase))
        # Force value to be in range of interpolator:
        if new_phase < 0.0:
            new_phase = 360 + new_phase
        if new_phase > max(self.ph):
            new_phase = new_phase % 360.0
        self.phase_val = new_phase
        new_voltage = self.ph_interp(new_phase)
        self.dac_dev.write_attribute("".join(("channel", str(self.dac_channel))), new_voltage)


if __name__ == "__main__":
    pt.server.server_run((PhaseshifterDS,))
