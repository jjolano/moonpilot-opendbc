import abc
import unittest

from opendbc.car.toyota.values import ToyotaSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests import common


class LateralEngageSafetyTest(common.SafetyTestBase, abc.ABC):
  """
  Moonpilot's lateral-engagement permission (controls_allowed_lateral), shared by every brand
  that opts in through its safety param.

  The enclosing class must provide `self.safety`, `self.packer`, `self._rx`, `self._tx`,
  `self._steer_tx_msg()` (a steering message that is only legal while steering is permitted)
  and `self._accel_tx_msg()` (an acceleration message, which the permission must never enable).
  The rx-check set and the enable both come from the safety param, so every test drives the
  real hooks rather than the globals: `_arm_lateral` is the car's own path to the state.
  """

  EPS_SCALE: int
  # The safety param flags the enclosing class sets up with, minus the lateral-engagement flag
  BASE_FLAGS = ToyotaSafetyFlags(0)

  @classmethod
  def setUpClass(cls):
    # Like SafetyTestBase: the mixin is collected whenever a test module imports it by name, and
    # it has no safety mode of its own to run against
    if cls.__name__ == "LateralEngageSafetyTest":
      cls.safety = None
      raise unittest.SkipTest
    super().setUpClass()

  def _set_safety_hooks(self, *extra_flags):
    param = int(self.BASE_FLAGS)
    for flag in extra_flags:
      param |= int(flag)
    self.safety.set_safety_hooks(CarParams.SafetyModel.toyota, self.EPS_SCALE | param)

  def _acc_main_msg(self, main_on: bool):
    return self.packer.make_can_msg_safety("PCM_CRUISE_2", 0, {"MAIN_ON": int(main_on)})

  def _arm_lateral(self):
    """Turn the cruise main switch on with openpilot engaged, i.e. the driver's own path."""
    self.safety.set_heartbeat_engaged(True)
    self.assertTrue(self._rx(self._acc_main_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    # The permission is additive: stock ACC is not set, so upstream's flag stays down
    self.assertFalse(self.safety.get_controls_allowed())

  def tearDown(self):
    # Leave the module-level globals as a stock config would, so the inherited upstream tests
    # (which only reason about controls_allowed) run against upstream's rules
    self.safety.set_controls_allowed_lateral(False)
    self.safety.set_heartbeat_engaged(False)

  def test_lateral_engage_inert_without_flag(self):
    """A stock safety param carries the enable as false, so the permission never rises"""
    self._set_safety_hooks()
    self.safety.set_heartbeat_engaged(True)
    self.assertTrue(self._rx(self._acc_main_msg(True)))

    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._steer_tx_msg()))
    self.assertFalse(self._tx(self._accel_tx_msg()))
    self._set_safety_hooks(ToyotaSafetyFlags.LATERAL_ENGAGE)

  def test_lateral_engage_arms_on_acc_main_and_heartbeat(self):
    """Arms on the rising edge of either input, and holds for as long as both stay true"""
    self._arm_lateral()

    # Persistence: a cycle with no edge keeps the permission
    self.safety.lateral_engage_update(True, True, False)
    self.assertTrue(self.safety.get_controls_allowed_lateral())

    # Heartbeat loss is not an edge — openpilot disengaging ends it outright
    self.safety.lateral_engage_update(True, False, False)
    self.assertFalse(self.safety.get_controls_allowed_lateral())

    # Re-arms on the heartbeat's own rising edge, with the main switch already on
    self.safety.lateral_engage_update(True, True, False)
    self.assertTrue(self.safety.get_controls_allowed_lateral())

    # Main switch off ends it, and its own rising edge arms it again
    self.safety.lateral_engage_update(False, True, False)
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.safety.lateral_engage_update(True, True, False)
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_lateral_engage_disarms_on_steering_disengage(self):
    """A steering override wins over the permission, and only the next edge re-arms.

    `steering_disengage` is upstream's, and only Tesla's rx hook sets it, so no Toyota can reach
    this state on the road — the harness injects it to hold the rule itself.
    """
    self._arm_lateral()

    self.safety.lateral_engage_update(True, True, True)
    self.assertFalse(self.safety.get_controls_allowed_lateral())

    # Held: the same state next cycle is not an edge
    self.safety.lateral_engage_update(True, True, True)
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_lateral_engage_exit(self):
    """The lag/validity path in safety_tick drops the permission along with controls_allowed"""
    self._arm_lateral()

    self.safety.set_controls_allowed(True)
    self.safety.set_timer(int(2e6))
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

    # And directly, since the exit is what safety_tick calls
    self.safety.set_controls_allowed_lateral(True)
    self.safety.lateral_engage_exit()
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_lateral_engage_init_clears(self):
    """A safety mode change resets the permission, and only a brand init can re-enable it"""
    self._arm_lateral()

    self._set_safety_hooks(ToyotaSafetyFlags.LATERAL_ENGAGE)
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._set_safety_hooks()

  def test_the_plan_end_to_end_sequence(self):
    """The whole feature in one sequence: main switch on permits steering and nothing else.

    Steering goes through, acceleration stays blocked (which is the split that keeps the car's own
    ACC in charge of speed), a brake press leaves steering alone, and the main switch off takes it
    all back.
    """
    self.safety.set_heartbeat_engaged(True)
    self.assertTrue(self._rx(self._acc_main_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self.safety.get_controls_allowed())

    self.assertTrue(self._tx(self._steer_tx_msg()))
    self.assertFalse(self._tx(self._accel_tx_msg()))

    self._rx(self._user_brake_msg(1))
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._tx(self._steer_tx_msg()))

    self._rx(self._acc_main_msg(False))
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._steer_tx_msg()))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_lateral_engage_allows_steering_only(self):
    """Steering is permitted while the permission stands; nothing longitudinal is"""
    self._arm_lateral()

    self.assertTrue(self._tx(self._steer_tx_msg()))
    self.assertFalse(self._tx(self._accel_tx_msg()))
    self.assertFalse(self.safety.get_longitudinal_allowed())

    # A brake press is the point of the feature: it leaves the steering alone
    self._rx(self._user_brake_msg(1))
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._tx(self._steer_tx_msg()))

    # But the steering message is still illegal once the permission is gone
    self._rx(self._acc_main_msg(False))
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._steer_tx_msg()))
