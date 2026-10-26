package autopilot.core;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class BrakeControllerTest {

    private static final double MAX_PRESSURE = 100.0;
    private static final double ABS_PRESSURE_THRESHOLD = 80.0;
    private static final double ABS_SPEED_THRESHOLD_KMH = 30.0;
    private static final double ABS_HIGH_PULSE = 90.0;
    private static final double ABS_LOW_PULSE = 60.0;
    private static final double NORMAL_PRESSURE = 40.0;
    private static final double PRESSURE_TOLERANCE = 1.0e-6;
    private static final String PARK_MESSAGE = "braking is unavailable while the gearbox is in park";
    private static final String NULL_GEAR_MESSAGE = "gear must not be null";
    private static final String INVALID_PRESSURE_MESSAGE = "pressure must be a finite non-negative value";
    private static final String INVALID_SPEED_MESSAGE = "speed must be a finite non-negative value";

    static Stream<Arguments> appliedPressureCases() {
        return Stream.of(
                Arguments.of(0.0, 0.0, GearBox.Gear.D, 0.0),
                Arguments.of(-0.0, 10.0, GearBox.Gear.N, 0.0),
                Arguments.of(0.1, 10.0, GearBox.Gear.R, 0.1),
                Arguments.of(NORMAL_PRESSURE, 100.0, GearBox.Gear.N, NORMAL_PRESSURE),
                Arguments.of(ABS_PRESSURE_THRESHOLD - 0.1, 100.0, GearBox.Gear.D, ABS_PRESSURE_THRESHOLD - 0.1),
                Arguments.of(ABS_PRESSURE_THRESHOLD, 100.0, GearBox.Gear.R, ABS_PRESSURE_THRESHOLD),
                Arguments.of(ABS_PRESSURE_THRESHOLD + 0.1, 0.0, GearBox.Gear.D, ABS_PRESSURE_THRESHOLD + 0.1),
                Arguments.of(ABS_PRESSURE_THRESHOLD + 0.1, -0.0, GearBox.Gear.N, ABS_PRESSURE_THRESHOLD + 0.1),
                Arguments.of(ABS_PRESSURE_THRESHOLD + 0.1, 0.1, GearBox.Gear.R, ABS_PRESSURE_THRESHOLD + 0.1),
                Arguments.of(ABS_PRESSURE_THRESHOLD + 0.1, ABS_SPEED_THRESHOLD_KMH - 0.1, GearBox.Gear.D, ABS_PRESSURE_THRESHOLD + 0.1),
                Arguments.of(ABS_PRESSURE_THRESHOLD + 0.1, ABS_SPEED_THRESHOLD_KMH, GearBox.Gear.N, ABS_PRESSURE_THRESHOLD + 0.1),
                Arguments.of(MAX_PRESSURE, ABS_SPEED_THRESHOLD_KMH, GearBox.Gear.D, MAX_PRESSURE),
                Arguments.of(MAX_PRESSURE + 0.1, 10.0, GearBox.Gear.R, MAX_PRESSURE),
                Arguments.of(150.0, ABS_SPEED_THRESHOLD_KMH, GearBox.Gear.N, MAX_PRESSURE),
                Arguments.of(Double.MAX_VALUE, 10.0, GearBox.Gear.D, MAX_PRESSURE),
                Arguments.of(ABS_PRESSURE_THRESHOLD + 0.1, ABS_SPEED_THRESHOLD_KMH + 0.1, GearBox.Gear.R, ABS_HIGH_PULSE),
                Arguments.of(MAX_PRESSURE, ABS_SPEED_THRESHOLD_KMH + 0.1, GearBox.Gear.D, ABS_HIGH_PULSE),
                Arguments.of(150.0, 50.0, GearBox.Gear.D, ABS_HIGH_PULSE),
                Arguments.of(Double.MAX_VALUE, Double.MAX_VALUE, GearBox.Gear.N, ABS_HIGH_PULSE)
        );
    }

    static Stream<Double> invalidPressures() {
        return Stream.of(-0.1, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY);
    }

    static Stream<Double> invalidSpeeds() {
        return Stream.of(-0.1, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY);
    }

    // A new controller stores zero pressure and has not raised the warning.
    @Test
    void newController_startsAtZeroPressureWithoutWarning() {
        BrakeController controller = new BrakeController();

        assertEquals(0.0, controller.getPressure(), PRESSURE_TOLERANCE);
        assertFalse(controller.isWarningActive());
    }

    // apply: one request on a fresh controller. Pressure at or below 80, or speed at or below 30,
    // is stored as requested and capped at 100. Crossing both thresholds returns the first ABS pulse, 90.
    // Pressure above 100 is bounded before the ABS check. Warning stays off.
    // D, N, and R are accepted; each invocation uses a fresh controller so the pulse starts high.
    @ParameterizedTest(name = "[{index}] pressure={0}, speed={1}, gear={2}")
    @MethodSource("appliedPressureCases")
    void apply_singleRequest_returnsBoundedOrFirstAbsPulse(double requestedPressure,
                                                           double speedKmh,
                                                           GearBox.Gear gear,
                                                           double expectedPressure) {
        BrakeController controller = new BrakeController();

        double actualPressure = controller.apply(requestedPressure, speedKmh, gear);

        assertEquals(expectedPressure, actualPressure, PRESSURE_TOLERANCE);
        assertEquals(expectedPressure, controller.getPressure(), PRESSURE_TOLERANCE);
        assertFalse(controller.isWarningActive());
    }

    // apply ABS branch: nextAbsPulseIsHigh flips on each qualifying call,
    // so the pulses are 90, then 60, then 90 again.
    @Test
    void apply_repeatedAbsRequests_alternatesHighThenLowPulse() {
        BrakeController controller = new BrakeController();

        double first = controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.D);
        assertEquals(ABS_HIGH_PULSE, first, PRESSURE_TOLERANCE);
        assertEquals(ABS_HIGH_PULSE, controller.getPressure(), PRESSURE_TOLERANCE);

        double second = controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.D);
        assertEquals(ABS_LOW_PULSE, second, PRESSURE_TOLERANCE);
        assertEquals(ABS_LOW_PULSE, controller.getPressure(), PRESSURE_TOLERANCE);

        double third = controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.D);
        assertEquals(ABS_HIGH_PULSE, third, PRESSURE_TOLERANCE);
        assertEquals(ABS_HIGH_PULSE, controller.getPressure(), PRESSURE_TOLERANCE);
        assertFalse(controller.isWarningActive());
    }

    // apply non-ABS branch: a request that misses the ABS condition sets the next pulse back to high,
    // so the following ABS call returns 90 rather than 60.
    @Test
    void apply_nonAbsCallAfterAbs_resetsNextPulseToHigh() {
        BrakeController controller = new BrakeController();
        controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.N);

        double held = controller.apply(NORMAL_PRESSURE, 50.0, GearBox.Gear.N);
        double nextAbsPulse = controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.N);

        assertEquals(NORMAL_PRESSURE, held, PRESSURE_TOLERANCE);
        assertEquals(ABS_HIGH_PULSE, nextAbsPulse, PRESSURE_TOLERANCE);
        assertEquals(ABS_HIGH_PULSE, controller.getPressure(), PRESSURE_TOLERANCE);
    }

    // apply non-ABS branch after the low pulse has already been delivered:
    // the next qualifying call is high again, rather than another low pulse.
    @Test
    void apply_nonAbsCallAfterLowPulse_resetsNextPulseToHigh() {
        BrakeController controller = new BrakeController();
        controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.D);
        controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.D);

        double held = controller.apply(NORMAL_PRESSURE, 20.0, GearBox.Gear.R);

        assertEquals(NORMAL_PRESSURE, held, PRESSURE_TOLERANCE);
        assertEquals(NORMAL_PRESSURE, controller.getPressure(), PRESSURE_TOLERANCE);

        double nextAbsPulse = controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.D);
        assertEquals(ABS_HIGH_PULSE, nextAbsPulse, PRESSURE_TOLERANCE);
        assertEquals(ABS_HIGH_PULSE, controller.getPressure(), PRESSURE_TOLERANCE);
    }

    // apply -> validatePressure: a negative or non-finite pressure is rejected.
    // Stored pressure stays put, and the pending low ABS pulse is neither consumed nor reset.
    @ParameterizedTest(name = "[{index}] pressure={0}")
    @MethodSource("invalidPressures")
    void apply_invalidPressure_throwsAndPreservesPressureAndAbsPulse(double requestedPressure) {
        BrakeController controller = primeAbsLowPulse();

        IllegalArgumentException exception = assertThrows(IllegalArgumentException.class,
                () -> controller.apply(requestedPressure, 50.0, GearBox.Gear.N));

        assertEquals(INVALID_PRESSURE_MESSAGE, exception.getMessage());
        assertEquals(ABS_HIGH_PULSE, controller.getPressure(), PRESSURE_TOLERANCE);
        assertFalse(controller.isWarningActive());
        assertEquals(ABS_LOW_PULSE, controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.N), PRESSURE_TOLERANCE);
    }

    // apply -> validateSpeed: a negative or non-finite speed is rejected.
    // Stored pressure stays put, and the pending low ABS pulse is neither consumed nor reset.
    @ParameterizedTest(name = "[{index}] speed={0}")
    @MethodSource("invalidSpeeds")
    void apply_invalidSpeed_throwsAndPreservesPressureAndAbsPulse(double speedKmh) {
        BrakeController controller = primeAbsLowPulse();

        IllegalArgumentException exception = assertThrows(IllegalArgumentException.class,
                () -> controller.apply(ABS_HIGH_PULSE, speedKmh, GearBox.Gear.D));

        assertEquals(INVALID_SPEED_MESSAGE, exception.getMessage());
        assertEquals(ABS_HIGH_PULSE, controller.getPressure(), PRESSURE_TOLERANCE);
        assertFalse(controller.isWarningActive());
        assertEquals(ABS_LOW_PULSE, controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.D), PRESSURE_TOLERANCE);
    }

    // apply -> ensureBrakingAllowed: gear P throws IllegalStateException before pressure or the ABS pulse changes.
    @Test
    void apply_park_throwsIllegalStateAndKeepsPressure() {
        BrakeController controller = primeAbsLowPulse();

        IllegalStateException exception = assertThrows(IllegalStateException.class,
                () -> controller.apply(MAX_PRESSURE, 50.0, GearBox.Gear.P));

        assertEquals(PARK_MESSAGE, exception.getMessage());
        assertEquals(ABS_HIGH_PULSE, controller.getPressure(), PRESSURE_TOLERANCE);
        assertFalse(controller.isWarningActive());
        assertEquals(ABS_LOW_PULSE, controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.D), PRESSURE_TOLERANCE);
    }

    // emergencyBrake -> ensureBrakingAllowed: gear P is rejected before pressure, warning, or the ABS pulse changes.
    @Test
    void emergencyBrake_park_throwsIllegalStateAndKeepsPressure() {
        BrakeController controller = controllerWithWarningAndPendingLowPulse();

        IllegalStateException exception = assertThrows(IllegalStateException.class,
                () -> controller.emergencyBrake(GearBox.Gear.P));

        assertEquals(PARK_MESSAGE, exception.getMessage());
        assertEquals(ABS_HIGH_PULSE, controller.getPressure(), PRESSURE_TOLERANCE);
        assertTrue(controller.isWarningActive());
        assertEquals(ABS_LOW_PULSE, controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.R), PRESSURE_TOLERANCE);
        assertTrue(controller.isWarningActive());
    }

    // apply -> ensureBrakingAllowed: a null gear throws NullPointerException and leaves pressure and the ABS pulse.
    @Test
    void apply_nullGear_throwsNullPointerExceptionAndKeepsPressure() {
        BrakeController controller = primeAbsLowPulse();

        NullPointerException exception = assertThrows(NullPointerException.class,
                () -> controller.apply(NORMAL_PRESSURE, 50.0, null));

        assertEquals(NULL_GEAR_MESSAGE, exception.getMessage());
        assertEquals(ABS_HIGH_PULSE, controller.getPressure(), PRESSURE_TOLERANCE);
        assertFalse(controller.isWarningActive());
        assertEquals(ABS_LOW_PULSE, controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.N), PRESSURE_TOLERANCE);
    }

    // emergencyBrake -> ensureBrakingAllowed: a null gear is rejected before pressure, warning, or the ABS pulse changes.
    @Test
    void emergencyBrake_nullGear_throwsNullPointerException() {
        BrakeController controller = new BrakeController();

        NullPointerException exception = assertThrows(NullPointerException.class,
                () -> controller.emergencyBrake(null));

        assertEquals(NULL_GEAR_MESSAGE, exception.getMessage());
        assertEquals(0.0, controller.getPressure(), PRESSURE_TOLERANCE);
        assertFalse(controller.isWarningActive());

        controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.D);
        exception = assertThrows(NullPointerException.class, () -> controller.emergencyBrake(null));

        assertEquals(NULL_GEAR_MESSAGE, exception.getMessage());
        assertEquals(ABS_HIGH_PULSE, controller.getPressure(), PRESSURE_TOLERANCE);
        assertFalse(controller.isWarningActive());
        assertEquals(ABS_LOW_PULSE, controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.D), PRESSURE_TOLERANCE);
    }

    // emergencyBrake: an allowed gear sets pressure to 100, turns the warning on, and resets the ABS pulse to high.
    // A later apply does not clear the warning. Repeating emergency braking keeps pressure and the warning.
    @Test
    void emergencyBrake_driveGear_setsFullPressureWarningAndResetsPulse() {
        BrakeController controller = new BrakeController();
        controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.D);

        controller.emergencyBrake(GearBox.Gear.D);
        controller.emergencyBrake(GearBox.Gear.N);
        controller.emergencyBrake(GearBox.Gear.R);

        assertEquals(MAX_PRESSURE, controller.getPressure(), PRESSURE_TOLERANCE);
        assertTrue(controller.isWarningActive());

        double nextAbsPulse = controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.D);
        assertEquals(ABS_HIGH_PULSE, nextAbsPulse, PRESSURE_TOLERANCE);
        assertEquals(ABS_HIGH_PULSE, controller.getPressure(), PRESSURE_TOLERANCE);
        assertTrue(controller.isWarningActive());
    }

    // The first qualifying apply leaves the next pulse low. A failed request must not move that flag.
    private static BrakeController primeAbsLowPulse() {
        BrakeController controller = new BrakeController();
        double applied = controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.D);
        assertEquals(ABS_HIGH_PULSE, applied, PRESSURE_TOLERANCE);
        return controller;
    }

    // Warning is already on, and the next ABS pulse is low, so a failed emergencyBrake can be observed
    // if it clears the warning or forces the following pulse back to high.
    private static BrakeController controllerWithWarningAndPendingLowPulse() {
        BrakeController controller = new BrakeController();
        controller.emergencyBrake(GearBox.Gear.D);
        double applied = controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.D);
        assertEquals(ABS_HIGH_PULSE, applied, PRESSURE_TOLERANCE);
        assertTrue(controller.isWarningActive());
        return controller;
    }
}
