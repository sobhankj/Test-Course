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

    @ParameterizedTest
    @CsvSource({
            "80, 0, 100, EMERGENCY_BRAKE, 100",
            "80, 20, 100, EMERGENCY_BRAKE, 100",
            "80, 30, 100, BRAKE, 25",
            "80, 40, 90, HOLD, 0",
            "70, 50, 100, ACCELERATE, 5",
            "98, 60, 100, ACCELERATE, 2",
            "100, 60, 100, HOLD, 0",
            "110, 60, 100, HOLD, 0"
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

    @Test
    void update_distanceBetweenSafeAndReleaseWhileBraking_keepsBrakeAtHysteresisPressure() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(SPEED_KMH, (EMERGENCY_DISTANCE_METERS + SAFE_DISTANCE_METERS) / 2.0);

        AdaptiveCruise.Decision decision = cruise.update(SPEED_KMH, (SAFE_DISTANCE_METERS + RELEASE_DISTANCE_METERS) / 2.0);

        assertEquals(AdaptiveCruise.Mode.BRAKE, decision.mode());
        assertEquals(HYSTERESIS_BRAKE_PRESSURE, decision.amount(), AMOUNT_TOLERANCE);
    }

    @Test
    void update_distanceAtSafeWhileBraking_keepsBrakeAtHysteresisPressure() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(SPEED_KMH, EMERGENCY_DISTANCE_METERS + 10.0);

        AdaptiveCruise.Decision decision = cruise.update(SPEED_KMH, SAFE_DISTANCE_METERS);

        assertEquals(AdaptiveCruise.Mode.BRAKE, decision.mode());
        assertEquals(HYSTERESIS_BRAKE_PRESSURE, decision.amount(), AMOUNT_TOLERANCE);
    }

    @Test
    void update_distanceAtReleaseWhileBelowTarget_clearsBrakingAndAccelerates() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(SPEED_KMH, EMERGENCY_DISTANCE_METERS + 10.0);

        AdaptiveCruise.Decision decision = cruise.update(SPEED_KMH, RELEASE_DISTANCE_METERS);

        assertEquals(AdaptiveCruise.Mode.ACCELERATE, decision.mode());
        assertEquals(MAX_ACCELERATION_STEP_KMH, decision.amount(), AMOUNT_TOLERANCE);
    }

    @Test
    void safeDistanceFor_zeroAndRepresentativeSpeed_returnsHalfSpeedInMeters() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        assertEquals(0.0, cruise.safeDistanceFor(0.0), AMOUNT_TOLERANCE);
        assertEquals(SAFE_DISTANCE_METERS, cruise.safeDistanceFor(SPEED_KMH), AMOUNT_TOLERANCE);
    }

    @Test
    void safeDistanceFor_negativeOrNonFiniteSpeed_throwsIllegalArgumentException() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        assertThrows(IllegalArgumentException.class, () -> cruise.safeDistanceFor(-1.0));
        assertThrows(IllegalArgumentException.class, () -> cruise.safeDistanceFor(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> cruise.safeDistanceFor(Double.POSITIVE_INFINITY));
    }

    @Test
    void update_invalidSpeed_throwsIllegalArgumentExceptionWithoutBlockingLaterUpdate() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        assertThrows(IllegalArgumentException.class, () -> cruise.update(-1.0, SAFE_DISTANCE_METERS + 10.0));
        assertThrows(IllegalArgumentException.class, () -> cruise.update(Double.NaN, SAFE_DISTANCE_METERS + 10.0));

        AdaptiveCruise.Decision decision = cruise.update(SPEED_KMH, SAFE_DISTANCE_METERS + 10.0);
        assertEquals(AdaptiveCruise.Mode.ACCELERATE, decision.mode());
        assertEquals(MAX_ACCELERATION_STEP_KMH, decision.amount(), AMOUNT_TOLERANCE);
    }

    @Test
    void update_invalidDistance_throwsAndPreservesBrakingState() {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
        cruise.update(SPEED_KMH, EMERGENCY_DISTANCE_METERS + 10.0);

        assertThrows(IllegalArgumentException.class, () -> cruise.update(SPEED_KMH, -1.0));
        assertThrows(IllegalArgumentException.class, () -> cruise.update(SPEED_KMH, Double.NaN));

        AdaptiveCruise.Decision decision = cruise.update(SPEED_KMH, (SAFE_DISTANCE_METERS + RELEASE_DISTANCE_METERS) / 2.0);
        assertEquals(AdaptiveCruise.Mode.BRAKE, decision.mode());
        assertEquals(HYSTERESIS_BRAKE_PRESSURE, decision.amount(), AMOUNT_TOLERANCE);
    }
}
