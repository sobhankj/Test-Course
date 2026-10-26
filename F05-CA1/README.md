# F05-CA1 — Autonomous Vehicle Control Simulation

This project is a Java model of autonomous-vehicle control logic: gear changes, steering, braking, adaptive cruise, and their coordination on each time tick. It has no real hardware, user interface, or time-simulation loop. Sensors are interfaces, and instantaneous readings are supplied through `SensorSnapshot`.

The project is built with Maven. Production code is in `src/main/java` and JUnit 5 tests are in `src/test/java`.

## Requirements

| Tool | Version |
| --- | --- |
| JDK | 26 (`maven.compiler.release` is 26) |
| Maven | 3.9 or newer |
| JUnit Jupiter | 5.10.2 (test scope only) |

If `java -version` is not 26, point the environment at a JDK 26 before running Maven. An example Temurin installation is:

```powershell
$env:JAVA_HOME = "C:\Program Files\Eclipse Adoptium\jdk-26.0.2.101-hotspot"
$env:Path = "$env:JAVA_HOME\bin;$env:Path"
```

## Build and test

Run the commands from this directory (`F05-CA1`), where `pom.xml` lives.

```powershell
mvn test
```

`mvn clean test` deletes `target` and runs the full suite again. Surefire is configured with `useModulePath` set to `false`. The runtime has no dependencies beyond the JDK.

## Package layout

```
src/main/java/autopilot
├── core          control logic
├── sensor        sensor interfaces and SensorSnapshot
└── exception     domain exceptions
src/test/java/autopilot/core
└── unit tests for each core class
```

| Package | Classes |
| --- | --- |
| `autopilot.core` | `AutoDriver`, `AdaptiveCruise`, `BrakeController`, `GearBox`, `SteeringController` |
| `autopilot.sensor` | `SpeedSensor`, `ObstacleSensor`, `LaneSensor`, `SensorSnapshot` |
| `autopilot.exception` | `IllegalGearChangeException`, `SensorFailureException` |

## One tick

`AutoDriver` is the coordinator. Its constructor takes the three sensors and the cruise target speed, then creates `SteeringController`, `BrakeController`, and `AdaptiveCruise` itself. None of the sensors may be null.

`tick(GearBox.Gear gear)` does the following:

1. Reads speed (km/h), obstacle distance (meters), and lane offset from the lane center (meters).
2. Updates steering from the lane offset.
3. Asks adaptive cruise for a decision based on speed and distance.
4. Calls `BrakeController` when the decision is `BRAKE` or `EMERGENCY_BRAKE`. For `ACCELERATE` and `HOLD` it returns brake pressure `0` and does not touch the brake controller.
5. Returns a `DriveResult`: steering angle, brake pressure, cruise mode, brake-warning flag, and `safeMode`.

A lane offset greater than `+0.5` m steers to `-45` degrees. An offset less than `-0.5` m steers to `+45` degrees. Otherwise `selfCenter()` is called.

### Safe mode

If any of the three sensor reads throws `SensorFailureException`, `tick` swallows it and returns:

- steering angle `0.0`
- brake pressure `30.0`
- cruise mode `BRAKE`
- warning inactive
- `safeMode` equal to `true`

This path does not call the steering or brake controllers. Their stored angle and pressure stay as they were. Only the values returned for this tick are the safe values above. Other exceptions, such as a null gear or braking while the gear is `P`, are not caught.

`warningActive` is `true` only after `emergencyBrake` has completed successfully. Safe mode does not turn that flag on.

## Classes

### GearBox

The initial gear is `P`. The gears are `P`, `R`, `N`, and `D`. Speed comes from a `SpeedSensor`. Changing to the current gear does nothing. A null sensor and a null target gear throw `NullPointerException`. A non-finite or negative speed throws `IllegalArgumentException`. A disallowed change throws `IllegalGearChangeException` and leaves the gear unchanged.

| From | To | Speed condition |
| --- | --- | --- |
| `P` | `R` | strictly below 5 km/h |
| `R` | `N` | any valid speed |
| `N` | `D` | any valid speed |
| `R` | `D` | exactly `0.0` |
| `D` | `R` | exactly `0.0` |

Every other pair is rejected, including `P` to `N` or `D`, and `D` to `N`. Entering reverse from park uses a strict `< 5` comparison, so exactly 5 km/h is rejected. `setSpeedSensor` replaces the sensor and does not accept null.

### SteeringController

The current angle and the target angle both start at zero.

| Constant | Value |
| --- | --- |
| Physical limit | ±450 degrees |
| High-speed limit | ±90 degrees |
| High-speed threshold | 120 km/h, using a strict `>` comparison |
| Maximum change per call | 60 degrees |

`update(requestedAngleDegrees, speedKmh)` validates both inputs first. The angle must be finite. The speed must be finite and non-negative; zero is valid. An invalid input throws `IllegalArgumentException` and changes neither angle.

After validation, the target is clamped to the limit allowed at that speed. At 120 km/h and below the limit is ±450 degrees. Only a speed strictly above 120 km/h reduces it to ±90 degrees. The current angle then moves toward the target by at most 60 degrees. When the remaining gap is 60 degrees or less, that call reaches the target and does not pass it.

`selfCenter()` sets the target to zero immediately and moves the current angle toward zero under the same 60-degree limit. `getCurrentAngleDegrees()` and `getTargetAngleDegrees()` do not change state. The two values are not always equal: the target is stored immediately, and the current angle approaches it one step at a time.

### BrakeController

Pressure starts at zero and the warning is off.

`apply(requestedPressure, speedKmh, gear)` rejects a non-finite or negative pressure or speed. A null gear throws `NullPointerException`. Gear `P` throws `IllegalStateException`. In both failure cases the previous pressure is kept.

The requested pressure is capped at 100. If the capped pressure is greater than 80 and the speed is strictly greater than 30 km/h, ABS alternates between 90 and 60. The first pulse is 90. When the ABS condition is not met, the capped pressure is stored and the next pulse is reset to 90. Pressure 80 and speed 30 do not themselves enter ABS.

`emergencyBrake(gear)` uses the same gear restriction, sets pressure to 100, turns the warning on, and resets the next ABS pulse to 90. That call does not apply an ABS pulse. A later `apply` does not turn the warning off.

### AdaptiveCruise

The target speed must be finite and non-negative. Otherwise the constructor throws `IllegalArgumentException`.

The safe distance is `speedKmh * 0.5` meters. The emergency distance is half of that, and the release distance is `1.2` times the safe distance. `update` rejects an invalid speed or distance the same way as the other classes, and it does not change the internal braking flag when validation fails.

| Condition | Result |
| --- | --- |
| distance ≤ emergency distance | `EMERGENCY_BRAKE` with amount `100` |
| already braking and distance is still below the release distance | `BRAKE`; proportional pressure while still inside the safe distance, otherwise hysteresis pressure `10` |
| distance < safe distance | `BRAKE` with pressure `(safe - distance) / safe * 100` |
| distance > safe distance and speed is below the target | `ACCELERATE` by the minimum of `5` and the gap to the target speed |
| otherwise | `HOLD` with amount `0` |

A distance exactly equal to the safe distance neither brakes nor accelerates; the result is `HOLD`. The `braking` flag stays on after a brake or emergency brake until distance reaches the release threshold. That avoids rapid switching between acceleration and braking. When the safe distance is zero, because the vehicle is stopped, the proportional pressure is 100.

`AutoDriver` does not connect an `ACCELERATE` amount to the brake or to a throttle actuator. It only reports that mode in `DriveResult.cruiseMode()` and returns brake pressure zero.

### Sensors and exceptions

`SpeedSensor`, `ObstacleSensor`, and `LaneSensor` each expose one read method.

`SensorSnapshot.of(speed, distance, laneOffset)` builds a healthy reading and implements all three interfaces, so the same instance can be passed for all three `AutoDriver` sensor parameters. `SensorSnapshot.failed(message)` makes every read throw `SensorFailureException` with that message. A null message is rejected by `failed`.

`IllegalGearChangeException` and `SensorFailureException` are both `RuntimeException` subclasses.

## Tests

| Test class | What it covers |
| --- | --- |
| `GearBoxTest` | Initial park state, allowed and rejected shifts, the 5 km/h and zero-speed boundaries, sensor replacement, and an unchanged gear after a failure |
| `SteeringControllerTest` | The 60-degree step, ±450 and ±90 clamping, the strict 120 km/h boundary, stepping back from an angle above 90 when speed crosses the threshold, self-centering without passing zero, and preserved state after invalid input |
| `BrakeControllerTest` | The pressure cap, ABS boundaries, pulse alternation, emergency braking, park, and preserved state after invalid input |
| `AdaptiveCruiseTest` | Safe distance, modes, hysteresis, distance boundaries, and input validation |
| `AutoDriverTest` | Lane correction, passing a cruise decision through to the brake, and the safe-mode result when a sensor fails |

The tests do not touch private fields. They check behavior through the public API.
