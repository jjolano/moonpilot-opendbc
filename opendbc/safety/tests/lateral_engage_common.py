import abc
import unittest

from opendbc.safety.tests import common


class LateralEngageSafetyTest(common.SafetyTestBase, abc.ABC):
  """
  Moonpilot's lateral-engagement permission (controls_allowed_lateral), shared by every brand
  that opts in through its safety param.

  The enclosing class must provide `self.safety`, `self.packer`, `self._rx`, `self._tx`,
  `self._steer_tx_msg()` (a steering message that is only legal while steering is permitted),
  `self._accel_tx_msg()` (an acceleration message, which the permission must never enable), and
  two brand hooks:

    `_set_lateral_engage_hooks(enabled)` — install this brand's hooks with the permission off or
    on. The permission is enabled *by* the brand init reading the safety param, so this is the
    only way to arm it and the tests drive the real path rather than the globals. Installed with
    `set_safety_hooks` rather than the globals so the clearing in `lateral_engage_init`, which
    runs before the brand init, is exercised too.

    `_acc_main_msg(main_on)` — this brand's cruise main switch message, the signal the permission
    keys on.

  Nothing here knows a brand: the flag bit, the safety model, any non-flag part of the param and
  the main-switch message all come from the enclosing class, so a new brand is one class that
  supplies them and no test changes. Both hooks are deliberately *not* stubbed here: this mixin is
  first in every concrete class's MRO, so a stub would shadow the brand's implementation rather
  than be overridden by it.
  """

  @classmethod
  def setUpClass(cls):
    # Like SafetyTestBase: the mixin is collected whenever a test module imports it by name, and
    # it has no safety mode of its own to run against
    if cls.__name__ == "LateralEngageSafetyTest":
      cls.safety = None
      raise unittest.SkipTest
    super().setUpClass()

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
    self._set_lateral_engage_hooks(False)
    self.safety.set_heartbeat_engaged(True)
    self.assertTrue(self._rx(self._acc_main_msg(True)))

    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._steer_tx_msg()))
    self.assertFalse(self._tx(self._accel_tx_msg()))
    self._set_lateral_engage_hooks(True)

  def test_lateral_engage_arms_on_acc_main_and_heartbeat(self):
    """The main switch arms it; the heartbeat only has to keep arriving"""
    self._arm_lateral()

    # Persistence: a cycle with no edge keeps the permission
    self.safety.lateral_engage_update(True, True, False)
    self.assertTrue(self.safety.get_controls_allowed_lateral())

    # A heartbeat that stops is not an edge: it clears only once the grace window has passed
    self.safety.lateral_engage_update(True, False, False)
    self.assertTrue(self.safety.get_controls_allowed_lateral())

    # Re-arms on the heartbeat's own rising edge, with the main switch already on
    self.safety.lateral_engage_update(True, True, False)
    self.assertTrue(self.safety.get_controls_allowed_lateral())

    # Main switch off ends it, and its own rising edge arms it again
    self.safety.lateral_engage_update(False, True, False)
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.safety.lateral_engage_update(True, True, False)
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_lateral_engage_arms_without_the_heartbeat(self):
    """The driver's own switch arms it with no heartbeat at all, and a torque frame goes out.

    This is the recorded failure as a regression test. `heartbeat_engaged` rides pandad's 10 Hz USB
    heartbeat, so it can only be true a tick *after* openpilot engaged, while openpilot is already
    commanding torque. A frame this layer rejects resets `desired_torque_last`, and the rate limiter
    then blocks every later frame until the command comes back within MAX_RATE_UP of zero — in the
    log that cost the car 0.92 s of STEERING_LKA while openpilot believed it was steering, which the
    car's own lane-keeping ECU reads as a message dropout.
    """
    self.safety.set_timer(0)
    self.safety.set_heartbeat_engaged(False)
    self.assertTrue(self._rx(self._acc_main_msg(True)))

    self.assertTrue(self.safety.get_controls_allowed_lateral())
    # The permission is additive: stock ACC is not set, so upstream's flag stays down
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(self._steer_tx_msg()))

  def test_lateral_engage_heartbeat_grace(self):
    """A missing heartbeat disengages only once it has been gone for the grace window.

    The arm no longer waits for that bit, so its remaining job is to notice openpilot is gone —
    and that reading is up to a full period of pandad's 10 Hz heartbeat late.
    """
    grace_us = 300000  # HEARTBEAT_GRACE_US in lateral_engage.h, 3x pandad's 10 Hz heartbeat
    self.safety.set_timer(1_000_000)
    self.safety.set_heartbeat_engaged(False)
    self.assertTrue(self._rx(self._acc_main_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed_lateral())

    # A heartbeat refresh extends the window rather than restarting the arm
    self.safety.set_timer(1_000_000 + grace_us // 2)
    self.safety.lateral_engage_update(True, True, False)
    self.assertTrue(self.safety.get_controls_allowed_lateral())

    # Silent for exactly the window: still armed
    self.safety.set_timer(1_000_000 + grace_us // 2 + grace_us)
    self.safety.lateral_engage_update(True, False, False)
    self.assertTrue(self.safety.get_controls_allowed_lateral())

    # One microsecond more: openpilot is gone, and the permission goes with it
    self.safety.set_timer(1_000_000 + grace_us // 2 + grace_us + 1)
    self.safety.lateral_engage_update(True, False, False)
    self.assertFalse(self.safety.get_controls_allowed_lateral())

    # openpilot coming back re-arms it on the heartbeat's own rising edge, the switch held on
    self.safety.set_timer(1_000_000 + grace_us // 2 + grace_us + 1000)
    self.safety.lateral_engage_update(True, True, False)
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_lateral_engage_disarms_on_steering_disengage(self):
    """A steering override wins over the permission, and only the next edge re-arms.

    `steering_disengage` is upstream's, and only Tesla's rx hook sets it, so no car in this
    feature's scope can reach this state on the road — the harness injects it to hold the rule
    itself.
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

    self._set_lateral_engage_hooks(True)
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self._set_lateral_engage_hooks(False)

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
