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

    static Stream<Arguments> appliedPressureCases() {
        return Stream.of(
                Arguments.of(0.0, 0.0, GearBox.Gear.D, 0.0),
                Arguments.of(NORMAL_PRESSURE, 100.0, GearBox.Gear.N, NORMAL_PRESSURE),
                Arguments.of(ABS_PRESSURE_THRESHOLD, 100.0, GearBox.Gear.R, ABS_PRESSURE_THRESHOLD),
                Arguments.of(MAX_PRESSURE, ABS_SPEED_THRESHOLD_KMH, GearBox.Gear.D, MAX_PRESSURE),
                Arguments.of(150.0, ABS_SPEED_THRESHOLD_KMH, GearBox.Gear.N, MAX_PRESSURE),
                Arguments.of(ABS_PRESSURE_THRESHOLD + 0.1, ABS_SPEED_THRESHOLD_KMH + 0.1, GearBox.Gear.R, ABS_HIGH_PULSE),
                Arguments.of(150.0, 50.0, GearBox.Gear.D, ABS_HIGH_PULSE)
        );
    }

    // apply: one request on a fresh controller. Pressure at or below 80, or speed at or below 30,
    // is returned as-is and capped at 100. Crossing both thresholds returns the first ABS pulse, 90.
    // Warning stays off.
    @ParameterizedTest
    @MethodSource("appliedPressureCases")
    void apply_singleRequest_returnsBoundedOrFirstAbsPulse(double requestedPressure,
                                                           double speedKmh,
                                                           GearBox.Gear gear,
                                                           double expectedPressure) {
        BrakeController controller = new BrakeController();

        double actualPressure = controller.apply(requestedPressure, speedKmh, gear);

        assertEquals(expectedPressure, actualPressure, PRESSURE_TOLERANCE);
        assertFalse(controller.isWarningActive());
    }

    // apply ABS branch: nextAbsPulseIsHigh flips on each qualifying call,
    // so the pulses are 90, then 60, then 90 again.
    @Test
    void apply_repeatedAbsRequests_alternatesHighThenLowPulse() {
        BrakeController controller = new BrakeController();

        double first = controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.D);
        double second = controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.D);
        double third = controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.D);

        assertEquals(ABS_HIGH_PULSE, first, PRESSURE_TOLERANCE);
        assertEquals(ABS_LOW_PULSE, second, PRESSURE_TOLERANCE);
        assertEquals(ABS_HIGH_PULSE, third, PRESSURE_TOLERANCE);
    }

    // apply non-ABS branch: a request that misses the ABS condition sets the next pulse back to high,
    // so the following ABS call returns 90 rather than 60.
    @Test
    void apply_nonAbsCallAfterAbs_resetsNextPulseToHigh() {
        BrakeController controller = new BrakeController();
        controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.N);

        controller.apply(NORMAL_PRESSURE, 50.0, GearBox.Gear.N);
        double nextAbsPulse = controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.N);

        assertEquals(ABS_HIGH_PULSE, nextAbsPulse, PRESSURE_TOLERANCE);
    }

    // apply -> ensureBrakingAllowed: gear P throws IllegalStateException before pressure changes.
    @Test
    void apply_park_throwsIllegalStateAndKeepsPressure() {
        BrakeController controller = new BrakeController();
        controller.apply(NORMAL_PRESSURE, 10.0, GearBox.Gear.D);

        IllegalStateException exception = assertThrows(IllegalStateException.class,
                () -> controller.apply(MAX_PRESSURE, 10.0, GearBox.Gear.P));

        assertEquals(PARK_MESSAGE, exception.getMessage());
        assertEquals(NORMAL_PRESSURE, controller.getPressure(), PRESSURE_TOLERANCE);
        assertFalse(controller.isWarningActive());
    }

    // emergencyBrake -> ensureBrakingAllowed: gear P is rejected before pressure or the warning flag changes.
    @Test
    void emergencyBrake_park_throwsIllegalStateAndKeepsPressure() {
        BrakeController controller = new BrakeController();
        controller.apply(NORMAL_PRESSURE, 10.0, GearBox.Gear.R);

        IllegalStateException exception = assertThrows(IllegalStateException.class,
                () -> controller.emergencyBrake(GearBox.Gear.P));

        assertEquals(PARK_MESSAGE, exception.getMessage());
        assertEquals(NORMAL_PRESSURE, controller.getPressure(), PRESSURE_TOLERANCE);
        assertFalse(controller.isWarningActive());
    }

    // apply -> ensureBrakingAllowed: a null gear throws NullPointerException and leaves the previous pressure.
    @Test
    void apply_nullGear_throwsNullPointerExceptionAndKeepsPressure() {
        BrakeController controller = new BrakeController();
        controller.apply(NORMAL_PRESSURE, 10.0, GearBox.Gear.D);

        NullPointerException exception = assertThrows(NullPointerException.class,
                () -> controller.apply(NORMAL_PRESSURE, 10.0, null));

        assertEquals(NULL_GEAR_MESSAGE, exception.getMessage());
        assertEquals(NORMAL_PRESSURE, controller.getPressure(), PRESSURE_TOLERANCE);
    }

    // emergencyBrake -> ensureBrakingAllowed: a null gear is rejected before pressure or warning is set.
    @Test
    void emergencyBrake_nullGear_throwsNullPointerException() {
        BrakeController controller = new BrakeController();

        NullPointerException exception = assertThrows(NullPointerException.class,
                () -> controller.emergencyBrake(null));

        assertEquals(NULL_GEAR_MESSAGE, exception.getMessage());
        assertEquals(0.0, controller.getPressure(), PRESSURE_TOLERANCE);
        assertFalse(controller.isWarningActive());
    }

    // apply -> validatePressure and validateSpeed: negative or non-finite values are rejected
    // before the stored pressure changes.
    @Test
    void apply_invalidPressureOrSpeed_throwsAndKeepsPressure() {
        BrakeController controller = new BrakeController();
        controller.apply(NORMAL_PRESSURE, 10.0, GearBox.Gear.N);

        assertThrows(IllegalArgumentException.class, () -> controller.apply(-0.1, 10.0, GearBox.Gear.N));
        assertThrows(IllegalArgumentException.class, () -> controller.apply(Double.NaN, 10.0, GearBox.Gear.N));
        assertThrows(IllegalArgumentException.class, () -> controller.apply(NORMAL_PRESSURE, -0.1, GearBox.Gear.N));
        assertThrows(IllegalArgumentException.class,
                () -> controller.apply(NORMAL_PRESSURE, Double.POSITIVE_INFINITY, GearBox.Gear.N));

        assertEquals(NORMAL_PRESSURE, controller.getPressure(), PRESSURE_TOLERANCE);
        assertFalse(controller.isWarningActive());
    }

    // emergencyBrake: an allowed gear sets pressure to 100, turns the warning on, and resets the ABS pulse to high.
    // A later apply does not clear the warning.
    @Test
    void emergencyBrake_driveGear_setsFullPressureWarningAndResetsPulse() {
        BrakeController controller = new BrakeController();
        controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.D);

        controller.emergencyBrake(GearBox.Gear.D);

        assertEquals(MAX_PRESSURE, controller.getPressure(), PRESSURE_TOLERANCE);
        assertTrue(controller.isWarningActive());

        double nextAbsPulse = controller.apply(ABS_HIGH_PULSE, 50.0, GearBox.Gear.D);
        assertEquals(ABS_HIGH_PULSE, nextAbsPulse, PRESSURE_TOLERANCE);
        assertTrue(controller.isWarningActive());
    }
}
