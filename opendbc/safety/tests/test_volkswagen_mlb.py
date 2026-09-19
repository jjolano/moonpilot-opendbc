#!/usr/bin/env python3
import unittest
from opendbc.car.volkswagen.values import VolkswagenSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
import opendbc.safety.tests.lateral_engage_common as lateral_engage_common  # moonpilot: lateral engagement coverage
from opendbc.safety.tests.common import CANPackerSafety

MSG_LS_01 = 0x10B       # TX by OP, ACC control buttons for cancel/resume
MSG_HCA_01 = 0x126      # TX by OP, Heading Control Assist steering torque
MSG_LDW_02 = 0x397      # TX by OP, Lane line recognition and text alerts


class TestVolkswagenMlbSafetyBase(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_01, MSG_LDW_02)}

  MAX_RATE_UP = 9
  MAX_RATE_DOWN = 10
  MAX_TORQUE_LOOKUP = [0], [300]
  MAX_RT_DELTA = 169

  DRIVER_TORQUE_ALLOWANCE = 60
  DRIVER_TORQUE_FACTOR = 3

  # Wheel speeds _esp_03_msg
  def _speed_msg(self, speed):
    values = {"ESP_%s_Radgeschw" % s: speed for s in ["HL", "HR", "VL", "VR"]}
    return self.packer.make_can_msg_safety("ESP_03", 0, values)

  # Driver brake pressure over threshold
  def _esp_05_msg(self, brake):
    values = {"ESP_Fahrer_bremst": brake}
    return self.packer.make_can_msg_safety("ESP_05", 0, values)

  # Brake pedal switch
  def _motor_03_msg(self, brake_signal=False, gas_signal=0):
    values = {
      "MO_Fahrer_bremst": brake_signal,
      "MO_Fahrpedalrohwert_01": gas_signal,
    }
    return self.packer.make_can_msg_safety("Motor_03", 0, values)

  def _user_brake_msg(self, brake):
    return self._motor_03_msg(brake_signal=brake)

  def _user_gas_msg(self, gas):
    return self._motor_03_msg(gas_signal=gas)

  # ACC engagement status
  def _tsk_status_msg(self, enable):
    values = {"TSK_Status_GRA_ACC_02": 1 if enable else 0}
    return self.packer.make_can_msg_safety("TSK_04", 1, values)

  def _pcm_status_msg(self, enable):
    return self._tsk_status_msg(enable)

  # Driver steering input torque
  def _torque_driver_msg(self, torque):
    values = {"EPS_Lenkmoment": abs(torque), "EPS_VZ_Lenkmoment": torque < 0}
    return self.packer.make_can_msg_safety("LH_EPS_03", 0, values)

  # openpilot steering output torque
  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"HCA_01_LM_Offset": abs(torque),
              "HCA_01_LM_OffSign": torque < 0,
              "HCA_01_Sendestatus": steer_req,
              "HCA_01_Status_HCA": 7 if steer_req else 3}
    return self.packer.make_can_msg_safety("HCA_01", 0, values)

  # Cruise control buttons
  def _ls_01_msg(self, cancel=0, resume=0, _set=0, bus=2):
    values = {"LS_Abbrechen": cancel, "LS_Tip_Setzen": _set, "LS_Tip_Wiederaufnahme": resume}
    return self.packer.make_can_msg_safety("LS_01", bus, values)

  # Verify brake_pressed is true if either the switch or pressure threshold signals are true
  def test_redundant_brake_signals(self):
    test_combinations = [(True, True, True), (True, True, False), (True, False, True), (False, False, False)]
    for brake_pressed, motor_03_signal, esp_05_signal in test_combinations:
      self._rx(self._motor_03_msg(brake_signal=False))
      self._rx(self._esp_05_msg(False))
      self.assertFalse(self.safety.get_brake_pressed_prev())
      self._rx(self._motor_03_msg(brake_signal=motor_03_signal))
      self._rx(self._esp_05_msg(esp_05_signal))
      self.assertEqual(brake_pressed, self.safety.get_brake_pressed_prev(),
                       f"expected {brake_pressed=} with {motor_03_signal=} and {esp_05_signal=}")

  def test_torque_measurements(self):
    # TODO: make this test work with all cars
    self._rx(self._torque_driver_msg(50))
    self._rx(self._torque_driver_msg(-50))
    self._rx(self._torque_driver_msg(0))
    self._rx(self._torque_driver_msg(0))
    self._rx(self._torque_driver_msg(0))
    self._rx(self._torque_driver_msg(0))

    self.assertEqual(-50, self.safety.get_torque_driver_min())
    self.assertEqual(50, self.safety.get_torque_driver_max())

    self._rx(self._torque_driver_msg(0))
    self.assertEqual(0, self.safety.get_torque_driver_max())
    self.assertEqual(-50, self.safety.get_torque_driver_min())

    self._rx(self._torque_driver_msg(0))
    self.assertEqual(0, self.safety.get_torque_driver_max())
    self.assertEqual(0, self.safety.get_torque_driver_min())


class TestVolkswagenMlbStockSafety(TestVolkswagenMlbSafetyBase):
  TX_MSGS = [[MSG_HCA_01, 0], [MSG_LDW_02, 0], [MSG_LS_01, 0], [MSG_LS_01, 2]]
  FWD_BLACKLISTED_ADDRS = {2: [MSG_HCA_01, MSG_LDW_02]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  def setUp(self):
    self.packer = CANPackerSafety("vw_mlb")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenMlb, 0)
    self.safety.init_tests()

  def test_spam_cancel_safety_check(self):
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._ls_01_msg(cancel=1)))
    self.assertFalse(self._tx(self._ls_01_msg(resume=1)))
    self.assertFalse(self._tx(self._ls_01_msg(_set=1)))
    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._ls_01_msg(resume=1)))

  def test_cancel_button(self):
    # Disable on rising edge of cancel button
    self._rx(self._tsk_status_msg(False))
    self.safety.set_controls_allowed(1)
    self._rx(self._ls_01_msg(cancel=True, bus=0))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after cancel")


# moonpilot: lateral engagement, stock ACC only -- the configuration where the car's own ACC holds
# speed. MLB is an ARM_HOST brand: its rx hook decodes no cruise main switch at all, so openpilot's
# own engaged heartbeat is the whole arm.
class TestVolkswagenMlbLateralEngageBase(TestVolkswagenMlbStockSafety):
  def _set_lateral_engage_hooks(self, enabled):
    param = int(VolkswagenSafetyFlags.LATERAL_ENGAGE) if enabled else 0
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenMlb, param)

  def _set_lat_engage_hooks(self):
    self._set_lateral_engage_hooks(True)
    self.safety.init_tests()

  def _steer_tx_msg(self):
    """HCA_01 torque, seeded so the rate and driver-torque checks pass: the permit is the only
    variable left"""
    self.safety.set_torque_meas(100, 100)
    self.safety.set_desired_torque_last(100)
    self.safety.set_rt_torque_last(100)
    return self._torque_cmd_msg(100, steer_req=1)

  def _accel_tx_msg(self):
    """0x10D ACC_05, the car's own ACC acceleration request. MLB has no openpilot longitudinal
    message at all -- its tx list is steering, HUD and ACC buttons -- so what blocks acceleration
    here is upstream's tx list, not the permission."""
    return common.make_msg(0, 0x10D, 8)


class TestVolkswagenMlbLateralEngage(lateral_engage_common.LateralEngageSafetyTest, TestVolkswagenMlbLateralEngageBase):
  LATERAL_ENGAGE_ARM = "host"

  def setUp(self):
    super().setUp()
    self._set_lat_engage_hooks()


if __name__ == "__main__":
  unittest.main()
