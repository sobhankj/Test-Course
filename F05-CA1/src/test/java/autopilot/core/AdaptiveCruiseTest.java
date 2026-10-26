package autopilot.core;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class AdaptiveCruiseTest {

    private static final double SPEED_KMH = 80.0;
    private static final double TARGET_SPEED_KMH = 100.0;
    private static final double SAFE_DISTANCE_METERS = 40.0;
    private static final double EMERGENCY_DISTANCE_METERS = 20.0;
    private static final double RELEASE_DISTANCE_METERS = 48.0;
    private static final double HYSTERESIS_BRAKE_PRESSURE = 10.0;
    private static final double MAX_ACCELERATION_STEP_KMH = 5.0;
    private static final double AMOUNT_TOLERANCE = 1.0e-6;

    // update on a fresh controller, where braking is still false:
    // distance <= emergency returns EMERGENCY_BRAKE 100,
    // distance between emergency and safe uses proportionalBrake,
    // distance == safe returns HOLD even when speed is below the target,
    // distance > safe and speed < target returns ACCELERATE capped at 5,
    // speed equal to or above the target returns HOLD 0.
    // At speed 0 the safe and emergency distances are both 0, so distance 0 is an emergency
    // and any positive distance accelerates, capped at 5.
    @ParameterizedTest
    @CsvSource({
            "80, 0, 100, EMERGENCY_BRAKE, 100",
            "80, 20, 100, EMERGENCY_BRAKE, 100",
            "80, 30, 100, BRAKE, 25",
            "80, 40, 90, HOLD, 0",
            "70, 50, 100, ACCELERATE, 5",
            "98, 60, 100, ACCELERATE, 2",
            "100, 60, 100, HOLD, 0",
            "110, 60, 100, HOLD, 0",
            "0, 0, 100, EMERGENCY_BRAKE, 100",
            "0, 1, 100, ACCELERATE, 5"
    })
    void update_freshController_selectsModeAndAmount(double speedKmh,
                                                      double distanceMeters,
                                                      double targetSpeedKmh,
                                                      AdaptiveCruise.Mode expectedMode,
                                                      double expectedAmount) {
        AdaptiveCruise cruise = new AdaptiveCruise(targetSpeedKmh);

        AdaptiveCruise.Decision decision = cruise.update(speedKmh, distanceMeters);

        assertEquals(expectedMode, decision.mode());
        assertEquals(expectedAmount, decision.amount(), AMOUNT_TOLERANCE);
    }

    // update braking branch: after braking is set, a distance between safe and release
    // stays in BRAKE and uses the fixed hysteresis pressure of 10 instead of accelerating.
    @Test
    void update_distanceBetweenSafeAndReleaseWhileBraking_keepsBrakeAtHysteresisPressure() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(SPEED_KMH, (EMERGENCY_DISTANCE_METERS + SAFE_DISTANCE_METERS) / 2.0);

        AdaptiveCruise.Decision decision = cruise.update(SPEED_KMH, (SAFE_DISTANCE_METERS + RELEASE_DISTANCE_METERS) / 2.0);

        assertEquals(AdaptiveCruise.Mode.BRAKE, decision.mode());
        assertEquals(HYSTERESIS_BRAKE_PRESSURE, decision.amount(), AMOUNT_TOLERANCE);
    }

    // update braking branch: distance == safe is still below the release distance,
    // so the hysteresis pressure of 10 is returned rather than HOLD.
    @Test
    void update_distanceAtSafeWhileBraking_keepsBrakeAtHysteresisPressure() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(SPEED_KMH, EMERGENCY_DISTANCE_METERS + 10.0);

        AdaptiveCruise.Decision decision = cruise.update(SPEED_KMH, SAFE_DISTANCE_METERS);

        assertEquals(AdaptiveCruise.Mode.BRAKE, decision.mode());
        assertEquals(HYSTERESIS_BRAKE_PRESSURE, decision.amount(), AMOUNT_TOLERANCE);
    }

    // update: distance >= release clears the braking flag in the same call.
    // Because speed is still below the target, that call then returns ACCELERATE by at most 5.
    @Test
    void update_distanceAtReleaseWhileBelowTarget_clearsBrakingAndAccelerates() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(SPEED_KMH, EMERGENCY_DISTANCE_METERS + 10.0);

        AdaptiveCruise.Decision decision = cruise.update(SPEED_KMH, RELEASE_DISTANCE_METERS);

        assertEquals(AdaptiveCruise.Mode.ACCELERATE, decision.mode());
        assertEquals(MAX_ACCELERATION_STEP_KMH, decision.amount(), AMOUNT_TOLERANCE);
    }

    // update: a distance strictly above the release distance also clears braking.
    // The same call then follows the normal rule and accelerates because 50 m is beyond
    // the 40 m safe distance and 80 km/h is still 20 km/h below the 100 km/h target.
    @Test
    void update_distanceAboveReleaseWhileBelowTarget_clearsBrakingAndAccelerates() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(SPEED_KMH, EMERGENCY_DISTANCE_METERS + 10.0);

        AdaptiveCruise.Decision decision = cruise.update(SPEED_KMH, RELEASE_DISTANCE_METERS + 2.0);

        assertEquals(AdaptiveCruise.Mode.ACCELERATE, decision.mode());
        assertEquals(MAX_ACCELERATION_STEP_KMH, decision.amount(), AMOUNT_TOLERANCE);
    }

    // update: reaching the release distance clears braking, but acceleration also requires
    // speed strictly below the target. At 80 km/h with an 80 km/h target the same call holds.
    @Test
    void update_distanceAtReleaseWhileAtTargetSpeed_clearsBrakingAndHolds() {
        AdaptiveCruise cruise = new AdaptiveCruise(SPEED_KMH);
        cruise.update(SPEED_KMH, EMERGENCY_DISTANCE_METERS + 10.0);

        AdaptiveCruise.Decision decision = cruise.update(SPEED_KMH, RELEASE_DISTANCE_METERS);

        assertEquals(AdaptiveCruise.Mode.HOLD, decision.mode());
        assertEquals(0.0, decision.amount(), AMOUNT_TOLERANCE);
    }

    // update braking branch: once braking is set, a later distance still below the safe
    // distance keeps the proportional pressure. It must not switch to the hysteresis value of 10.
    // At 30 m with a 40 m safe distance the pressure is (40 - 30) / 40 * 100 = 25.
    @Test
    void update_distanceBelowSafeWhileBraking_keepsProportionalPressure() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(SPEED_KMH, 30.0);

        AdaptiveCruise.Decision decision = cruise.update(SPEED_KMH, 30.0);

        assertEquals(AdaptiveCruise.Mode.BRAKE, decision.mode());
        assertEquals(25.0, decision.amount(), AMOUNT_TOLERANCE);
    }

    // safeDistanceFor: a finite non-negative speed returns speed * 0.5 meters,
    // including zero speed.
    @Test
    void safeDistanceFor_zeroAndRepresentativeSpeed_returnsHalfSpeedInMeters() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        assertEquals(0.0, cruise.safeDistanceFor(0.0), AMOUNT_TOLERANCE);
        assertEquals(SAFE_DISTANCE_METERS, cruise.safeDistanceFor(SPEED_KMH), AMOUNT_TOLERANCE);
    }

    // safeDistanceFor -> validateSpeed: a negative, NaN, or infinite speed is rejected.
    @Test
    void safeDistanceFor_negativeOrNonFiniteSpeed_throwsIllegalArgumentException() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        assertThrows(IllegalArgumentException.class, () -> cruise.safeDistanceFor(-1.0));
        assertThrows(IllegalArgumentException.class, () -> cruise.safeDistanceFor(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> cruise.safeDistanceFor(Double.POSITIVE_INFINITY));
        assertThrows(IllegalArgumentException.class, () -> cruise.safeDistanceFor(Double.NEGATIVE_INFINITY));
    }

    // update -> validateSpeed: invalid speed is rejected before braking changes.
    // A later valid update can still accelerate.
    @Test
    void update_invalidSpeed_throwsIllegalArgumentExceptionWithoutBlockingLaterUpdate() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        assertThrows(IllegalArgumentException.class, () -> cruise.update(-1.0, SAFE_DISTANCE_METERS + 10.0));
        assertThrows(IllegalArgumentException.class, () -> cruise.update(Double.NaN, SAFE_DISTANCE_METERS + 10.0));

        AdaptiveCruise.Decision decision = cruise.update(SPEED_KMH, SAFE_DISTANCE_METERS + 10.0);
        assertEquals(AdaptiveCruise.Mode.ACCELERATE, decision.mode());
        assertEquals(MAX_ACCELERATION_STEP_KMH, decision.amount(), AMOUNT_TOLERANCE);
    }

    // update -> validateDistance: a negative or non-finite distance is rejected
    // and leaves the existing braking flag set, so the next valid call still uses hysteresis.
    @Test
    void update_invalidDistance_throwsAndPreservesBrakingState() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(SPEED_KMH, EMERGENCY_DISTANCE_METERS + 10.0);

        assertThrows(IllegalArgumentException.class, () -> cruise.update(SPEED_KMH, -1.0));
        assertThrows(IllegalArgumentException.class, () -> cruise.update(SPEED_KMH, Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> cruise.update(SPEED_KMH, Double.POSITIVE_INFINITY));
        assertThrows(IllegalArgumentException.class, () -> cruise.update(SPEED_KMH, Double.NEGATIVE_INFINITY));

        AdaptiveCruise.Decision decision = cruise.update(SPEED_KMH, (SAFE_DISTANCE_METERS + RELEASE_DISTANCE_METERS) / 2.0);
        assertEquals(AdaptiveCruise.Mode.BRAKE, decision.mode());
        assertEquals(HYSTERESIS_BRAKE_PRESSURE, decision.amount(), AMOUNT_TOLERANCE);
    }

    // constructor: a negative or non-finite target speed is rejected.
    @Test
    void constructor_negativeOrNonFiniteTargetSpeed_throwsIllegalArgumentException() {
        assertThrows(IllegalArgumentException.class, () -> new AdaptiveCruise(-1.0));
        assertThrows(IllegalArgumentException.class, () -> new AdaptiveCruise(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> new AdaptiveCruise(Double.POSITIVE_INFINITY));
        assertThrows(IllegalArgumentException.class, () -> new AdaptiveCruise(Double.NEGATIVE_INFINITY));
    }

    // constructor: zero is a valid target. With speed also zero and distance beyond the
    // zero safe distance, speed is not below the target, so the decision is HOLD 0.
    @Test
    void constructor_zeroTargetSpeed_isAcceptedAndHoldsAtZeroSpeed() {
        AdaptiveCruise cruise = new AdaptiveCruise(0.0);

        AdaptiveCruise.Decision decision = cruise.update(0.0, 1.0);

        assertEquals(AdaptiveCruise.Mode.HOLD, decision.mode());
        assertEquals(0.0, decision.amount(), AMOUNT_TOLERANCE);
    }
}
