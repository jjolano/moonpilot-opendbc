#!/usr/bin/env python3
import unittest

from opendbc.car.mg.values import MgSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
import opendbc.safety.tests.lateral_engage_common as lateral_engage_common  # moonpilot: by module, never by name -- see AGENTS.md


def checksum(msg):
  addr, dat, bus = msg
  ret = bytearray(dat)

  if addr in (0x1b6, 0x242):
    crc = 0xFF
    for byte in ret[:-1]:
      crc ^= byte
      for _ in range(8):
        crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    ret[-1] = crc ^ 0xFF

  return addr, ret, bus


class TestMGSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):

  TX_MSGS = [[0x1fd, 0], ]
  RELAY_MALFUNCTION_ADDRS = {0: (0x1fd,)}
  FWD_BLACKLISTED_ADDRS = {2: [0x1fd,]}

  MAX_RATE_UP = 6
  MAX_RATE_DOWN = 10
  MAX_TORQUE_LOOKUP = [0], [300]
  MAX_RT_DELTA = 125

  DRIVER_TORQUE_ALLOWANCE = 100
  DRIVER_TORQUE_FACTOR = 2

  def setUp(self):
    self.packer = CANPackerSafety("mg")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mg, 0)
    self.safety.init_tests()
    self.counters = {addr: 0 for addr in (0x1b6, 0x1ec, 0x23c, 0x242)}

  def _counter(self, addr):
    counter = self.counters[addr]
    self.counters[addr] = (counter + 1) % 16
    return counter

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAReqToqHSC2": torque, "LKAReqToqStsHSC2": steer_req}
    return self.packer.make_can_msg_safety("FVCM_HSC2_FrP03", 0, values)

  def _speed_msg(self, speed):
    values = {"VehSpdAvgHSC2": speed * 3.6, "VehSpdAvgAlvRCHSC2": self._counter(0x23c)}
    return self.packer.make_can_msg_safety("SCS_HSC2_FrP19", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"DrvrStrgDlvrdToqHSC2": torque * 0.01, "ChLKAAlvRCHSC2": self._counter(0x1ec)}
    return self.packer.make_can_msg_safety("EPS_HSC2_FrP03", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BrkPdlAppdHSC2": 1 if brake else 0, "BrkPdlAppdRCHSC2": self._counter(0x1b6)}
    return self.packer.make_can_msg_safety("EHBS_HSC2_FrP00", 0, values, fix_checksum=checksum)

  def _user_gas_msg(self, gas):
    values = {"EPTAccelActuPosHSC2": 100 if gas else 0}
    return self.packer.make_can_msg_safety("GW_HSC2_HCU_FrP00", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"ACCSysSts_RadarHSC2": 2 if enable else 1, "ACCSysAlvRlngCtr_SCSHSC2": self._counter(0x242)}
    return self.packer.make_can_msg_safety("RADAR_HSC2_FrP00", 0, values, fix_checksum=checksum)


# moonpilot: lateral engagement. Stock ACC only -- mg's mode enters controls on RADAR_HSC2_FrP00's
# ACCSysSts_RadarHSC2 and its tx set is the steering message alone -- and host-armed: that signal is
# the ACC state, not the cruise main switch, so openpilot's heartbeat is the arm.
class TestMGLateralEngageBase(TestMGSafety):
  def _steer_tx_msg(self):
    """A steering message that is only legal while steering is permitted"""
    self._set_prev_torque(100)
    return self._torque_cmd_msg(100, steer_req=1)

  def _accel_tx_msg(self):
    """RADAR_HSC2_FrP00: where this car's ACC status rides, and the message openpilot would have to
    write to command acceleration at all. The permission must not add it to the tx set."""
    values = {"ACCSysSts_RadarHSC2": 2, "ACCSysAlvRlngCtr_SCSHSC2": self._counter(0x242)}
    return self.packer.make_can_msg_safety("RADAR_HSC2_FrP00", 0, values, fix_checksum=checksum)

  def _set_lateral_engage_hooks(self, enabled):
    flags = int(MgSafetyFlags.LATERAL_ENGAGE) if enabled else 0
    self.safety.set_safety_hooks(CarParams.SafetyModel.mg, flags)

  def _set_lat_engage_hooks(self):
    """The brand base's own setup: install with the permission on, then let init run."""
    self._set_lateral_engage_hooks(True)
    self.safety.init_tests()


class TestMGLateralEngage(lateral_engage_common.LateralEngageSafetyTest, TestMGLateralEngageBase):
  LATERAL_ENGAGE_ARM = "host"

  def setUp(self):
    super().setUp()
    self._set_lat_engage_hooks()


if __name__ == "__main__":
  unittest.main()
