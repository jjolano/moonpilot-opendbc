import abc
import unittest

from opendbc.safety.tests import common

# HEARTBEAT_GRACE_US in moonpilot/lateral_engage.h: 3x pandad's 10 Hz heartbeat, and how stale a
# heartbeat reading may stand for openpilot being engaged before the permission is dropped.
LATERAL_ENGAGE_GRACE_US = 300000


class LateralEngageSafetyTest(common.SafetyTestBase, abc.ABC):
  """
  Moonpilot's lateral-engagement permission (controls_allowed_lateral), shared by every brand
  that opts in through its safety param.

  The enclosing class must provide `self.safety`, `self.packer`, `self._rx`, `self._tx`,
  `self._steer_tx_msg()` (a steering message that is only legal while steering is permitted),
  `self._accel_tx_msg()` (an acceleration message, which the permission must never enable), and
  one brand hook:

    `_set_lateral_engage_hooks(enabled)` — install this brand's hooks with the permission off or
    on. The permission is enabled *by* the brand init reading the safety param, so this is the
    only way to arm it and the tests drive the real path rather than the globals. Installed with
    `set_safety_hooks` rather than the globals so the clearing in `lateral_engage_init`, which
    runs before the brand init, is exercised too.

  It must also declare which arm its brand's mode uses, because the two modes arm from different
  evidence and one of them has no car signal at all:

    LATERAL_ENGAGE_ARM = "switch"  — the car's own cruise main switch arms it. Also needs
                                     `_acc_main_msg(main_on)`, that brand's main-switch message.
    LATERAL_ENGAGE_ARM = "host"    — openpilot's own engaged heartbeat arms it, level-armed. A
                                     class with this arm defines no `_acc_main_msg`: the brand mode
                                     passes `LATERAL_ENGAGE_ARM_HOST` and decodes no main switch.

  Nothing else here knows a brand: the flag bit, the safety model, any non-flag part of the param
  and the main-switch message all come from the enclosing class. The brand hook is deliberately
  *not* stubbed here: this mixin is first in every concrete class's MRO, so a stub would shadow
  the brand's implementation rather than be overridden by it.
  """

  LATERAL_ENGAGE_ARM = "switch"

  @classmethod
  def setUpClass(cls):
    # Like SafetyTestBase: the mixin is collected whenever a test module imports it by name, and
    # it has no safety mode of its own to run against
    if cls.__name__ == "LateralEngageSafetyTest":
      cls.safety = None
      raise unittest.SkipTest
    super().setUpClass()

  def _arm_signal(self, on: bool) -> None:
    """Present or withdraw whatever this brand's mode arms from, without asserting the outcome."""
    if self.LATERAL_ENGAGE_ARM == "switch":
      self.safety.set_heartbeat_engaged(on)
      self.assertTrue(self._rx(self._acc_main_msg(on)))
    else:
      self.safety.set_heartbeat_engaged(on)
      self.safety.lateral_engage_update(False, on, False)

  def _arm_lateral(self):
    """Turn the arm on with openpilot engaged, i.e. the driver's own path: the cruise main switch
    where the brand decodes it, the host's own claim where it does not."""
    self._arm_signal(True)
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    # The permission is additive: stock ACC is not set, so upstream's flag stays down
    self.assertFalse(self.safety.get_controls_allowed())

  def _disarm_lateral(self) -> None:
    """Take the arm's own signal away. Under ARM_HOST that is a heartbeat which stops, and the
    permission only goes once the grace window has passed — the cross-check that keeps a merely
    late heartbeat from taking the steering with it, not an accident of the arm."""
    if self.LATERAL_ENGAGE_ARM == "switch":
      self.assertTrue(self._rx(self._acc_main_msg(False)))
    else:
      self.safety.set_heartbeat_engaged(False)
      self.safety.set_timer(LATERAL_ENGAGE_GRACE_US + 1)
      self.safety.lateral_engage_update(False, False, False)

  def tearDown(self):
    # Leave the module-level globals as a stock config would, so the inherited upstream tests
    # (which only reason about controls_allowed) run against upstream's rules
    self.safety.set_controls_allowed_lateral(False)
    self.safety.set_heartbeat_engaged(False)

  def test_lateral_engage_inert_without_flag(self):
    """A stock safety param carries the enable as false, so the permission never rises"""
    self._set_lateral_engage_hooks(False)
    self._arm_signal(True)

    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._steer_tx_msg()))
    self.assertFalse(self._tx(self._accel_tx_msg()))
    self._set_lateral_engage_hooks(True)

  def test_lateral_engage_arms_on_acc_main_and_heartbeat(self):
    """ARM_SWITCH: the main switch arms it; the heartbeat only has to keep arriving"""
    if self.LATERAL_ENGAGE_ARM != "switch":
      self.skipTest("host-armed brand: no main switch to edge")
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

  def test_lateral_engage_arms_on_the_heartbeat(self):
    """ARM_HOST: openpilot's own heartbeat is the whole arm, and the permission is level-armed.

    A brand in this group decodes no cruise main switch, so there is nothing on the car to wait
    for: the host saying it is engaged is the evidence, and a grant that disappears — the lag exit
    below, a heartbeat that goes stale — comes back by itself on the next fresh heartbeat instead
    of waiting for a car signal to transition.
    """
    if self.LATERAL_ENGAGE_ARM != "host":
      self.skipTest("switch-armed brand: the main switch is the arm")
    self.safety.set_timer(0)
    self._arm_lateral()

    # No car message was needed to get here: steering is permitted and nothing longitudinal is
    self.assertTrue(self._tx(self._steer_tx_msg()))
    self.assertFalse(self._tx(self._accel_tx_msg()))

    # The lag/validity exit clears it, and one more fresh heartbeat brings it back
    self.safety.lateral_engage_exit()
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.safety.lateral_engage_update(False, True, False)
    self.assertTrue(self.safety.get_controls_allowed_lateral())

    # A heartbeat that went stale is the one thing that does drop it
    self.safety.set_heartbeat_engaged(False)
    self.safety.set_timer(LATERAL_ENGAGE_GRACE_US + 1000)
    self.safety.lateral_engage_update(False, False, False)
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._steer_tx_msg()))

  def test_lateral_engage_arms_without_the_heartbeat(self):
    """ARM_SWITCH: the driver's own switch arms it with no heartbeat at all, and a torque frame
    goes out.

    This is the recorded failure as a regression test. `heartbeat_engaged` rides pandad's 10 Hz USB
    heartbeat, so it can only be true a tick *after* openpilot engaged, while openpilot is already
    commanding torque. A frame this layer rejects resets `desired_torque_last`, and the rate limiter
    then blocks every later frame until the command comes back within MAX_RATE_UP of zero — in the
    log that cost the car 0.92 s of STEERING_LKA while openpilot believed it was steering, which the
    car's own lane-keeping ECU reads as a message dropout.
    """
    if self.LATERAL_ENGAGE_ARM != "switch":
      self.skipTest("host-armed brand: the heartbeat is the arm")
    self.safety.set_timer(0)
    self.safety.set_heartbeat_engaged(False)
    self.assertTrue(self._rx(self._acc_main_msg(True)))

    self.assertTrue(self.safety.get_controls_allowed_lateral())
    # The permission is additive: stock ACC is not set, so upstream's flag stays down
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(self._steer_tx_msg()))

  def test_lateral_engage_heartbeat_grace(self):
    """ARM_SWITCH: a missing heartbeat disengages only once it has been gone for the grace window.

    The arm no longer waits for that bit, so its remaining job is to notice openpilot is gone —
    and that reading is up to a full period of pandad's 10 Hz heartbeat late.
    """
    if self.LATERAL_ENGAGE_ARM != "switch":
      self.skipTest("host-armed brand: covered by test_lateral_engage_arms_on_the_heartbeat")
    grace_us = LATERAL_ENGAGE_GRACE_US
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
    """A steering override wins over the permission, and it holds for as long as it is asserted.

    `steering_disengage` is upstream's, and only Tesla's rx hook sets it: on a brand whose car does
    not raise it the harness injects it to hold the rule itself. It holds rather than edges because
    a host-armed car would otherwise take the permission straight back on the next heartbeat, with
    the driver still holding the wheel.
    """
    self._arm_lateral()
    acc_main = self.LATERAL_ENGAGE_ARM == "switch"

    self.safety.lateral_engage_update(acc_main, True, True)
    self.assertFalse(self.safety.get_controls_allowed_lateral())

    # Held: the driver is still on the wheel, so there is still no permission
    self.safety.lateral_engage_update(acc_main, True, True)
    self.assertFalse(self.safety.get_controls_allowed_lateral())

    # Released: the host-armed arm comes back on the next heartbeat by itself, while the
    # switch-armed one waits for its own switch edge -- the difference the two modes exist for
    self.safety.lateral_engage_update(acc_main, True, False)
    if self.LATERAL_ENGAGE_ARM == "switch":
      self.assertFalse(self.safety.get_controls_allowed_lateral())
      self.safety.lateral_engage_update(False, True, False)
      self.safety.lateral_engage_update(True, True, False)
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_lateral_engage_exit(self):
    """The lag/validity path in safety_tick drops the permission along with controls_allowed"""
    self._arm_lateral()

    self.safety.set_controls_allowed(True)
    self.safety.set_timer(int(2e6))
    self.safety.safety_tick()
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
    """The whole feature in one sequence: the arm permits steering and nothing else.

    Steering goes through, acceleration stays blocked (which is the split that keeps the car's own
    ACC in charge of speed), a brake press leaves steering alone, and the arm going away takes it
    all back.
    """
    self._arm_lateral()

    self.assertTrue(self._tx(self._steer_tx_msg()))
    self.assertFalse(self._tx(self._accel_tx_msg()))

    self._rx(self._user_brake_msg(1))
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._tx(self._steer_tx_msg()))

    self._disarm_lateral()
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
    self._disarm_lateral()
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._steer_tx_msg()))
