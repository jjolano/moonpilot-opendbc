#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
import opendbc.safety.tests.lateral_engage_common as lateral_engage_common  # moonpilot: lateral engagement coverage
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.car.psa.values import PsaSafetyFlags

LANE_KEEP_ASSIST = 0x3F2


class TestPsaSafetyBase(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (LANE_KEEP_ASSIST,)}
  FWD_BLACKLISTED_ADDRS = {2: [LANE_KEEP_ASSIST]}
  TX_MSGS = [[1010, 0]]

  MAIN_BUS = 0
  ADAS_BUS = 1
  CAM_BUS = 2

  STEER_ANGLE_MAX = 390
  DEG_TO_CAN = 10

  ANGLE_RATE_BP = [0., 5., 25.]
  ANGLE_RATE_UP = [2.5, 1.5, .2]
  ANGLE_RATE_DOWN = [5., 2., .3]

  def setUp(self):
    self.packer = CANPackerSafety("psa_aee2010_r3")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.psa, 0)
    self.safety.init_tests()

  def _angle_cmd_msg(self, angle: float, enabled: bool):
    values = {"SET_ANGLE": angle, "TORQUE_FACTOR": 100 if enabled else 0}
    return self.packer.make_can_msg_safety("LANE_KEEP_ASSIST", self.MAIN_BUS, values)

  def _angle_meas_msg(self, angle: float):
    values = {"ANGLE": angle}
    return self.packer.make_can_msg_safety("STEERING_ALT", self.MAIN_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"RVV_ACC_ACTIVATION_REQ": enable}
    return self.packer.make_can_msg_safety("HS2_DAT_MDD_CMD_452", self.ADAS_BUS, values)

  def _speed_msg(self, speed):
    values = {"VITESSE_VEHICULE_ROUES": speed * 3.6}
    return self.packer.make_can_msg_safety("HS2_DYN_ABR_38D", self.MAIN_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"P013_MainBrake": brake}
    return self.packer.make_can_msg_safety("Dat_BSI", self.CAM_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"P002_Com_rAPP": int(gas * 100)}
    return self.packer.make_can_msg_safety("Dyn_CMM", self.MAIN_BUS, values)

  def test_rx_hook(self):
    # speed
    for _ in range(10):
      self.assertTrue(self._rx(self._speed_msg(0)))
    msg = self._speed_msg(0)
    # invalidate checksum
    msg[0].data[5] = 0x00
    self.assertFalse(self._rx(msg))

    # cruise
    for _ in range(10):
      self.assertTrue(self._rx(self._pcm_status_msg(0)))
    msg = self._pcm_status_msg(0)
    # invalidate checksum
    msg[0].data[5] = 0x00
    self.assertFalse(self._rx(msg))
    msg = self._pcm_status_msg(0)
    # write to unused payload byte
    msg[0].data[6] = 0xAB
    self.assertTrue(self._rx(msg))


class TestPsaStockSafety(TestPsaSafetyBase):

  def setUp(self):
    self.packer = CANPackerSafety("psa_aee2010_r3")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.psa, 0)
    self.safety.init_tests()


# moonpilot: lateral engagement, stock ACC only -- the configuration where the car's own ACC holds
# speed. PSA is an ARM_HOST brand: psa.h decodes no cruise main switch, so openpilot's own engaged
# heartbeat is the whole arm (see moonpilot/lateral_engage.h).
class TestPsaLateralEngageBase(TestPsaStockSafety):
  def _set_lateral_engage_hooks(self, enabled):
    param = int(PsaSafetyFlags.LATERAL_ENGAGE) if enabled else 0
    self.safety.set_safety_hooks(CarParams.SafetyModel.psa, param)

  def _set_lat_engage_hooks(self):
    self._set_lateral_engage_hooks(True)
    self.safety.init_tests()

  def _steer_tx_msg(self):
    """LANE_KEEP_ASSIST with a live angle request: legal only while steering is permitted"""
    return self._angle_cmd_msg(0, True)

  def _accel_tx_msg(self):
    """0x2B6 HS2_DYN1_MDD_ETAT_2B6, the car's own ACC deceleration request. PSA's tx list is
    steering and nothing else, and the platform has no openpilot longitudinal mode to add one, so
    what blocks acceleration here is upstream's tx list, not the permission."""
    return common.make_msg(self.MAIN_BUS, 0x2B6, 8)


class TestPsaLateralEngage(lateral_engage_common.LateralEngageSafetyTest, TestPsaLateralEngageBase):
  LATERAL_ENGAGE_ARM = "host"

  def setUp(self):
    super().setUp()
    self._set_lat_engage_hooks()


if __name__ == "__main__":
    unittest.main()
