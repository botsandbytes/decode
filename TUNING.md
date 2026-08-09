# Casablanca Tuning Guide

Scope: tuning the chassis driver-assist layer (`utilities/Casablanca.java`) — friction
compensation, input smoothing, heading lock, and goal-zone protection. This assumes
PedroPathing's own `Constants.java` tuning (drivetrain PIDFs, localizer, max velocity,
centripetal scaling, heading PIDF, predictive braking coefficients) is already done — Casablanca
reuses those values directly, so redo the calibrations below any time `Constants.java` changes.

## The three-bucket mental model

Every tunable in Casablanca is one of three kinds. Knowing which kind you're touching tells you
*how* to tune it:

- **Measured** — a physical constant. There is a correct number and an `OpMode` that finds it.
  Don't guess these, don't feel-tune them away from the measured value.
- **Derived** — computed at runtime from a Measured value or from Pedro's own already-tuned
  constants (`Constants.followerConstants`, `Constants.driveConstants`). Nothing to tune directly.
- **Feel** — a judgment call (driver comfort, risk tolerance). No formula produces these. Tune by
  driving and feeling, on the field, ideally with the actual competition driver.

## Recommended order

**Phase 1 — Measure.** One field session, one OpMode: `FrictionCalibrationOpMode`.

1. `A` — kS X (`casablanca.friction.x`)
2. `B` — kS Y (`casablanca.friction.y`)
3. `X` — rotation, three phases in one pass: static breakaway (`friction.rot` /
   `heading_lock.ks_static`), breakaway while holding 50% forward (`heading_lock.ks_moving`), then
   100% turn power held ~1s for max angular velocity — telemetry prints a suggested
   `sentinel.rotation_lookahead_time` (`0.45 / maxOmega`).

Paste every number straight into `config.yaml`. This phase is almost entirely mechanical — read a
number, paste a number. Little judgment involved, and that's the point.

**Phase 2 — Push and sanity check.**

```bash
./gradlew pushConfig
```

Re-init any OpMode to pick it up, then check the boot log for Casablanca's `Init check:` line — it
compares your configured `depthHardStop` / `sideHardStop` against the physics-predicted stopping
distance at your measured top speed. If hard-stop is smaller than that number, that's a signal to
make a *deliberate* call about `depthHardStop`/`sideHardStop`, not an accident to leave alone.

**Phase 3 — Feel-tune, one variable at a time, in this order:**

1. **Input smoothing** first — every later judgment (heading lock's snappiness, protection's
   abruptness) is filtered through it, so get it right before you start judging anything else.
2. **Heading lock's feel half** (`intent_threshold`, `max_power`) — now that smoothing isn't
   muddying the signal.
3. **Zone protection feel** (`slow_down`, `decel_safety_factor`, `lane_blend_distance`,
   `wallRepulsionPower`) — approach goal zones at speed and angle, tune the protection curve.
4. Loop back to smoothing only if protection changes made stopping *feel* different.

**Discipline:** change one config value, `pushConfig`, drive, judge, repeat. Changing three things
between test drives means you can't attribute the result to anything. `./gradlew resetConfig` bails
back to the last committed-good config if a session goes sideways. The `@Configurable` fields
(anything public non-final in `Casablanca.java`) are live-draggable on the Panels Dashboard without
even a `pushConfig` round-trip — great for a first-pass "what range even feels right," but dashboard
edits don't persist. Once you land on a value, write it into `config.yaml` for real.

**Never feel-tune a Measured value away from its measured number.** If the feel is wrong after a
correct measurement, the bug is a Feel parameter or a real logic issue elsewhere — quietly
re-measuring a physical constant to "fix a feeling" just means re-discovering the same ghost next
season.

## Per-subsystem reference

### A. Friction compensation — `frictionX`, `frictionY`, `frictionRot`
**Bucket: Measured.** Tool: `FrictionCalibrationOpMode`, buttons A/B/X.
Ramps power on the given axis from zero until the follower detects real motion, reports the
breakaway power. Re-measure when wheels change, robot mass changes (new mechanism bolted on), or
before touching anything else if the robot "feels different than last season."
- Too low → dead zone, no response to light stick input.
- Too high → twitchy, snaps to a floor power on the lightest touch.

### B. Heading lock — measured half — `ks_static`, `ks_moving`, `moving_speed_threshold`, `rotation_lookahead_time`
**Bucket: Measured.** Same OpMode, same X-button pass as friction. `moving_speed_threshold` isn't
a free parameter either — set it near the forward speed you actually calibrated `ks_moving` at,
don't guess it independently.

### C. Heading lock — feel half — `intent_threshold`, `max_power`
**Bucket: Feel.** No OpMode. Tune last, on the field, with the real driver's hands on the sticks.
**Acceptance test:** drive straight, release the turn stick abruptly mid-turn. It should coast to
the release heading and hold — not snap, not buzz, not feel like something grabbed the wheel.
- `intent_threshold` too low → lock disengages on stick noise, feels twitchy.
- `intent_threshold` too high → fine steering inputs get swallowed, feels like the lock is fighting
  the driver.
- `max_power` too low → can't hold against real contact (opponent bump). Too high → a bump could
  spin the robot faster than intended — it's a safety clamp, don't raise it past what's needed.

### D. Input smoothing — `smoothTime`, `backLiftMultiplier`, `extraSmoothBackLift`
**Bucket: Feel.** No OpMode. Tune first among the feel parameters. Full-send stick slams in an open
area; raise `smoothTime` until wheelies/back-wheel-lift on hard stops go away, back off if it starts
feeling laggy. `backLiftMultiplier` only affects deceleration — isolate it by testing stop-from-speed
specifically. Good sign: if you can't tell it's there while driving normally, it's tuned right —
noticeable only when it's off, not when it's on.

### E. Zone protection — `depthSlowDown`, `depthHardStop`, `sideSlowDown`, `sideHardStop`, `decelSafetyFactor`, `laneBlendDistance`, `wallRepulsionPower`
**Bucket: mixed** — `hard_stop` is policy but boot-checked against physics; `slow_down` is feel but
constrained; `decel_safety_factor` is risk tolerance. Read the boot log first, then feel-tune.
- **Hard constraint:** `slow_down` must be *larger* than `hard_stop`, or it's dead code — the
  hard-stop check fires first and the slow-down ramp is never reached. (This exact bug shipped once:
  `depth.slow_down=8` under `depth.hard_stop=10`; fixed to `15`.) Starting rule of thumb:
  `slow_down = hard_stop × 1.5`.
- `decel_safety_factor` — how much you trust the physics model. `0.7` pads the required stopping
  distance by ~1.4×. Lower it (more padding) if you see clipping into a zone at speed in practice;
  raise it (less padding) if it's frustratingly conservative and stops you way early.
- `lane_blend_distance` — how far out protection starts nudging vs. feeling like a wall that
  appears suddenly. Bigger = smoother/earlier, smaller = sharper/later.
- `wallRepulsionPower` — only matters once already violating. Firm enough to visibly push out, not
  violent.
- Re-check the boot log any time top speed changes (new wheels, gear ratio, drivetrain rebuild) —
  required stopping distance scales with velocity squared.

## Troubleshooting matrix

| Symptom | Likely cause | What to do |
|---|---|---|
| Won't move on light stick input | `frictionX`/`Y` too low | Re-measure (Measured — don't guess) |
| Snaps/jumps on light stick input | `frictionX`/`Y` too high | Re-measure |
| Buzzes/jitters trying to hold heading | `ks` values wrong, or `intent_threshold` too low | Re-measure `ks` first, then feel-tune `intent_threshold` |
| Heading lock drifts, doesn't hold | `ks` too low | Re-measure. Don't casually raise the shared Pedro heading PIDF — it's shared with autonomous |
| Heading lock fights the driver | `intent_threshold` too low | Raise it |
| Tips / back wheels lift on hard stop | `backLiftMultiplier` too low | Raise it |
| Driving feels laggy | `smoothTime` too high | Lower it |
| Clips into a goal zone at speed | Check boot log: `hard_stop` below physics minimum? | If not, lower `decel_safety_factor` (more margin) |
| Protection nudges way too early | `lane_blend_distance` too large, or `decel_safety_factor` too conservative | Lower one |
| Turns blocked well before a zone edge | `rotation_lookahead_time` too high vs. measured max omega | Re-measure |
| Could spin past a zone edge before blocked | `rotation_lookahead_time` too low | Raise it — this is a safety parameter, err high if unsure |

**One more discipline note:** never change more than one config value between test drives.
Casablanca's subsystems interact — you can't attribute an improvement or regression to the right
knob if you changed three at once.
