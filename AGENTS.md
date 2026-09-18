# opendbc, as moonpilot uses it

This is `jjolano/moonpilot-opendbc`, a fork of `commaai/opendbc`. `master` is the fork branch and carries the fork's commits; `upstream` is commaai and stays there. Merge, never rebase, never force-push.

The fork's rules live in the superproject, and they apply here too:

@../AGENTS.md

## What the fork changed here

The safety layer, for the fork's lateral-only engagement: `opendbc/safety/moonpilot/lateral_engage.h` (the rule) and its declarations header, the twelve `controls_allowed` reads in `opendbc/safety/lateral.h` widened to `(controls_allowed || controls_allowed_lateral)`, `PCM_CRUISE_2.MAIN_ON` read into `acc_main_on` with a lateral-engagement rx-check set per branch in `opendbc/safety/modes/toyota.h`, and `ToyotaSafetyFlags.LATERAL_ENGAGE` in `opendbc/car/toyota/values.py`. The tests are `opendbc/safety/tests/lateral_engage_common.py` plus one class per rx-check branch in `opendbc/safety/tests/test_toyota.py`. Everything else is upstream's.

## The invariants

- **`controls_allowed` keeps upstream's exact meaning.** It is stock ACC engagement, it is the only thing `get_longitudinal_allowed()` reads, and every longitudinal check, the brake/gas/regen exits and the heartbeat logic depend on it staying that way. The fork steers through the *second*, additive flag. Widening the first one instead hands the fork actuators it does not have, which is the single mistake this fork exists to avoid making.
- **A stock safety param must stay byte-identical to upstream's.** The fork's rx-check sets are duplicated per branch rather than edited in place, so a car without the fork's flag keeps upstream's array untouched. Keep that shape.
- **`ToyotaSafetyFlags.LATERAL_ENGAGE` (`16 << 8`) and `TOYOTA_PARAM_LATERAL_ENGAGE` in `modes/toyota.h` are one wire contract** split across a Python enum and C. Change them together or not at all; a mismatch is a param the car never enables.
- **The enable rides the safety param, not `alternative_experience`.** `toyota_init` must see it to pick the rx-check set, and the param reaches `init` by construction while the alternative experience only happens to be set first by pandad.
- **Import the fork's test mixin by module, never by name.** `LateralEngageSafetyTest` is an ABC whose `setUpClass` skips when it is collected directly, and unittest collects every `TestCase` a module imports by name: `from ...lateral_engage_common import LateralEngageSafetyTest` puts the skipped ABC into `test_toyota.py`'s ids, where it sorts before the concrete `TestToyota*` classes and wins `mutation.py`'s one-id-per-(method, module) dedup. Every fork mutant then survives a suite that reports green. `import opendbc.safety.tests.lateral_engage_common as lateral_engage_common` and qualify the base classes.
- **The seam in `lateral.h` takes upstream's blank line, so no line below it moves.** `mutation.py` carries a line-keyed allowlist of known surviving mutants (`known_survivors`: `lateral.h` 188, 218 and 219). A line inserted above them shifts every number and the allowlist silently stops matching, which turns three upstream-known survivors into a red gate. Keep this file's line count identical to upstream's.
- **This tree is on openpilot's boot path.** `card`, `selfdrived` and `plannerd` import from here at start, so keep module-level imports light — the fork's init-path rule applies inside this repo too.

## Gates

`opendbc/safety/tests/test.sh` is the first of two. The second is upstream's `opendbc/safety/tests/mutation.py`, which comma's CI runs as the `Safety mutation tests` job and exits 1 on any mutant the suite leaves alive — so a fork test has to fail when the rule it covers is mutated, not merely pass. Both are run from `opendbc_repo` with this repo's own `.venv/bin/python`. Comma's rule for a fork that edits `opendbc/safety/` is that the whole suite is preserved and passes, and that script enforces it mechanically with 100% line coverage of every non-libsafety safety file. Three things fall out of that:

- **New C is unreachable from the tests until it is exposed in the harness**: a new global or function needs a shim in `opendbc/safety/tests/libsafety/safety.c` and a declaration in `libsafety_py.py`'s `ffi.cdef`, or the coverage gate can never be satisfied.
- **The coverage data has to start clean.** `test.sh` removes `*.gcda` but not `*.gcno`, so stale objects from an earlier build make `gcovr` merge two line maps for one header and abort; remove both before a gate run that follows a source change.
- **MISRA runs from `opendbc/safety/tests/misra/test_misra.sh`** and needs `cppcheck`: run it with this repo's own `.venv/bin/python` so its default `import cppcheck` resolves, and set `CPPCHECK_DIR` to that package's `install` directory. A clean compile is not a clean check.

`ruff check .`, `ty check`, `codespell` and `cpplint` are the rest of `lefthook.yml`'s gate — and `codespell` only sees files git knows about, so `git add` first.
