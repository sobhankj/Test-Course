package autopilot.core;

import autopilot.exception.IllegalGearChangeException;
import autopilot.exception.SensorFailureException;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class GearBoxTest {

    private static final double REVERSE_ENTRY_SPEED_LIMIT_KMH = 5.0;
    private static final double BELOW_REVERSE_ENTRY_SPEED_KMH = 4.9;
    private static final double STOPPED_SPEED_KMH = 0.0;
    private static final double MOVING_SPEED_KMH = 40.0;

    // Initial state: currentGear starts at P before changeTo is called.
    @Test
    void getCurrentGear_beforeAnyShift_returnsPark() {
        GearBox gearBox = new GearBox(sensor(STOPPED_SPEED_KMH));

        assertEquals(GearBox.Gear.P, gearBox.getCurrentGear());
    }

    // changeTo -> isAllowed: P to R is accepted when speed is strictly below 5 km/h.
    @Test
    void changeTo_parkToReverseBelowFive_changesToReverse() {
        ConfigurableSpeedSensor sensor = sensor(BELOW_REVERSE_ENTRY_SPEED_KMH);
        GearBox gearBox = new GearBox(sensor);

        gearBox.changeTo(GearBox.Gear.R);

        assertEquals(GearBox.Gear.R, gearBox.getCurrentGear());
    }

    // changeTo -> isAllowed: R to N and N to D are allowed at any valid speed, including 40 km/h.
    @Test
    void changeTo_reverseToNeutralAndNeutralToDriveWhileMoving_reachesDrive() {
        ConfigurableSpeedSensor sensor = sensor(MOVING_SPEED_KMH);
        GearBox gearBox = gearboxIn(GearBox.Gear.R, sensor);
        sensor.setSpeedKmh(MOVING_SPEED_KMH);

        gearBox.changeTo(GearBox.Gear.N);
        assertEquals(GearBox.Gear.N, gearBox.getCurrentGear());

        gearBox.changeTo(GearBox.Gear.D);
        assertEquals(GearBox.Gear.D, gearBox.getCurrentGear());
    }

    // changeTo -> isAllowed: R to D is accepted only when speed is exactly 0.
    @Test
    void changeTo_reverseToDriveAtZero_changesToDrive() {
        GearBox gearBox = gearboxIn(GearBox.Gear.R, sensor(STOPPED_SPEED_KMH));

        gearBox.changeTo(GearBox.Gear.D);

        assertEquals(GearBox.Gear.D, gearBox.getCurrentGear());
    }

    // changeTo -> isAllowed: D to R is accepted only when speed is exactly 0.
    @Test
    void changeTo_driveToReverseAtZero_changesToReverse() {
        GearBox gearBox = gearboxIn(GearBox.Gear.D, sensor(STOPPED_SPEED_KMH));

        gearBox.changeTo(GearBox.Gear.R);

        assertEquals(GearBox.Gear.R, gearBox.getCurrentGear());
    }

    // changeTo -> isAllowed: P to R uses a strict less-than check. 0, the smallest positive
    // double, and the double immediately below 5 km/h are accepted.
    @ParameterizedTest(name = "speed {0} km/h")
    @MethodSource("speedsStrictlyBelowReverseEntryLimit")
    void changeTo_parkToReverseStrictlyBelowFive_changesToReverse(double speedKmh) {
        GearBox gearBox = new GearBox(sensor(speedKmh));

        gearBox.changeTo(GearBox.Gear.R);

        assertEquals(GearBox.Gear.R, gearBox.getCurrentGear());
    }

    // changeTo -> isAllowed: exactly 5 km/h, the next double above 5, and a speed well above 5
    // throw IllegalGearChangeException and leave the gear at P.
    @Test
    void changeTo_parkToReverseAtOrAboveFive_throwsAndStaysInPark() {
        assertRejected(GearBox.Gear.P, GearBox.Gear.R, REVERSE_ENTRY_SPEED_LIMIT_KMH);
        assertRejected(GearBox.Gear.P, GearBox.Gear.R, Math.nextUp(REVERSE_ENTRY_SPEED_LIMIT_KMH));
        assertRejected(GearBox.Gear.P, GearBox.Gear.R, REVERSE_ENTRY_SPEED_LIMIT_KMH + 10.0);
    }

    // changeTo -> isAllowed: R to D at a positive speed throws and stays in R.
    @Test
    void changeTo_reverseToDriveWhileMoving_throwsAndStaysInReverse() {
        assertRejected(GearBox.Gear.R, GearBox.Gear.D, BELOW_REVERSE_ENTRY_SPEED_KMH);
    }

    // changeTo -> isAllowed: D to R at a positive speed throws and stays in D.
    @Test
    void changeTo_driveToReverseWhileMoving_throwsAndStaysInDrive() {
        assertRejected(GearBox.Gear.D, GearBox.Gear.R, BELOW_REVERSE_ENTRY_SPEED_KMH);
    }

    // changeTo -> isAllowed: P to N, P to D, D to N, D to P, N to R, N to P, and R to P are not allowed at any speed.
    // Each rejection throws IllegalGearChangeException and keeps the current gear.
    @Test
    void changeTo_prohibitedTransitions_throwAndKeepCurrentGear() {
        assertRejected(GearBox.Gear.P, GearBox.Gear.N, STOPPED_SPEED_KMH);
        assertRejected(GearBox.Gear.P, GearBox.Gear.D, STOPPED_SPEED_KMH);
        assertRejected(GearBox.Gear.D, GearBox.Gear.N, STOPPED_SPEED_KMH);
        assertRejected(GearBox.Gear.D, GearBox.Gear.P, STOPPED_SPEED_KMH);
        assertRejected(GearBox.Gear.N, GearBox.Gear.R, STOPPED_SPEED_KMH);
        assertRejected(GearBox.Gear.N, GearBox.Gear.P, STOPPED_SPEED_KMH);
        assertRejected(GearBox.Gear.R, GearBox.Gear.P, STOPPED_SPEED_KMH);
    }

    // changeTo -> isAllowed: R to D and D to R compare speed with exactly 0.0.
    // Negative zero compares equal to 0.0, so both shifts are accepted.
    @ParameterizedTest(name = "{0} to {1}")
    @MethodSource("exactZeroTransitions")
    void changeTo_exactZeroTransitionAtNegativeZero_changesGear(GearBox.Gear from, GearBox.Gear to) {
        GearBox gearBox = gearboxIn(from, sensor(-0.0));

        gearBox.changeTo(to);

        assertEquals(to, gearBox.getCurrentGear());
    }

    // The smallest positive double is greater than 0, so R to D and D to R are rejected.
    @ParameterizedTest(name = "{0} to {1}")
    @MethodSource("exactZeroTransitions")
    void changeTo_exactZeroTransitionAtMinPositiveSpeed_throwsAndKeepsGear(GearBox.Gear from, GearBox.Gear to) {
        assertRejected(from, to, Double.MIN_VALUE);
    }

    // R to N and N to D have no upper speed bound. The largest finite double is still valid.
    @Test
    void changeTo_reverseToNeutralAndNeutralToDriveAtMaxFiniteSpeed_reachesDrive() {
        ConfigurableSpeedSensor sensor = sensor(Double.MAX_VALUE);
        GearBox gearBox = gearboxIn(GearBox.Gear.R, sensor);

        gearBox.changeTo(GearBox.Gear.N);
        assertEquals(GearBox.Gear.N, gearBox.getCurrentGear());

        gearBox.changeTo(GearBox.Gear.D);
        assertEquals(GearBox.Gear.D, gearBox.getCurrentGear());
    }

    // changeTo early return: requesting the current gear for every Gear value returns before the sensor is read,
    // so a failing sensor does not change the gear.
    @ParameterizedTest
    @EnumSource(GearBox.Gear.class)
    void changeTo_sameGear_keepsGearWithoutReadingSensor(GearBox.Gear gear) {
        ConfigurableSpeedSensor sensor = sensor(STOPPED_SPEED_KMH);
        GearBox gearBox = gearboxIn(gear, sensor);
        sensor.failWith(new SensorFailureException("speed sensor failed"));
        int readsBeforeShift = sensor.readCount();

        assertDoesNotThrow(() -> gearBox.changeTo(gear));

        assertEquals(readsBeforeShift, sensor.readCount());
        assertEquals(gear, gearBox.getCurrentGear());
    }

    // changeTo -> validateSpeed: a negative, NaN, or infinite sensor speed throws IllegalArgumentException
    // and leaves the gear at P.
    @Test
    void changeTo_negativeOrNonFiniteSpeed_throwsAndKeepsGear() {
        assertSpeedRejected(-0.1);
        assertSpeedRejected(-Double.MIN_VALUE);
        assertSpeedRejected(Double.NaN);
        assertSpeedRejected(Double.POSITIVE_INFINITY);
        assertSpeedRejected(Double.NEGATIVE_INFINITY);
    }

    // changeTo reads SpeedSensor only when the target gear differs. A SensorFailureException from that read
    // propagates and the gear stays at P.
    @Test
    void changeTo_sensorFailureDuringShift_throwsAndKeepsGear() {
        ConfigurableSpeedSensor sensor = sensor(STOPPED_SPEED_KMH);
        GearBox gearBox = new GearBox(sensor);
        sensor.failWith(new SensorFailureException("speed sensor failed"));

        assertThrows(SensorFailureException.class, () -> gearBox.changeTo(GearBox.Gear.R));

        assertEquals(GearBox.Gear.P, gearBox.getCurrentGear());
    }

    // changeTo: a null target throws NullPointerException before the sensor is read, so the gear stays at P.
    @Test
    void changeTo_nullTarget_throwsNullPointerExceptionAndKeepsPark() {
        ConfigurableSpeedSensor sensor = sensor(STOPPED_SPEED_KMH);
        GearBox gearBox = new GearBox(sensor);
        int readsBeforeShift = sensor.readCount();

        NullPointerException exception = assertThrows(NullPointerException.class, () -> gearBox.changeTo(null));

        assertEquals("targetGear must not be null", exception.getMessage());
        assertEquals(readsBeforeShift, sensor.readCount());
        assertEquals(GearBox.Gear.P, gearBox.getCurrentGear());
    }

    // The constructor rejects a null sensor before a gearbox can be used.
    @Test
    void constructor_nullSensor_throwsNullPointerException() {
        NullPointerException exception = assertThrows(NullPointerException.class, () -> new GearBox(null));

        assertEquals("speedSensor must not be null", exception.getMessage());
    }

    // setSpeedSensor(null) is rejected and the sensor already installed stays in use.
    @Test
    void setSpeedSensor_null_throwsAndKeepsPreviousSensor() {
        ConfigurableSpeedSensor original = sensor(STOPPED_SPEED_KMH);
        GearBox gearBox = new GearBox(original);

        NullPointerException exception = assertThrows(NullPointerException.class, () -> gearBox.setSpeedSensor(null));

        assertEquals("speedSensor must not be null", exception.getMessage());
        int readsBeforeShift = original.readCount();
        gearBox.changeTo(GearBox.Gear.R);
        assertEquals(readsBeforeShift + 1, original.readCount());
        assertEquals(GearBox.Gear.R, gearBox.getCurrentGear());
    }

    // A replacement sensor is the one read by the next shift.
    @Test
    void setSpeedSensor_replacement_isUsedByNextShift() {
        ConfigurableSpeedSensor original = sensor(MOVING_SPEED_KMH);
        ConfigurableSpeedSensor replacement = sensor(STOPPED_SPEED_KMH);
        GearBox gearBox = new GearBox(original);

        gearBox.setSpeedSensor(replacement);
        gearBox.changeTo(GearBox.Gear.R);

        assertEquals(0, original.readCount());
        assertEquals(1, replacement.readCount());
        assertEquals(GearBox.Gear.R, gearBox.getCurrentGear());
    }

    // A failure from the sensor installed by setSpeedSensor propagates and leaves the gear unchanged.
    @Test
    void setSpeedSensor_failingReplacement_throwsAndKeepsGear() {
        ConfigurableSpeedSensor original = sensor(STOPPED_SPEED_KMH);
        GearBox gearBox = gearboxIn(GearBox.Gear.R, original);
        ConfigurableSpeedSensor replacement = sensor(STOPPED_SPEED_KMH);
        replacement.failWith(new SensorFailureException("replacement sensor failed"));

        gearBox.setSpeedSensor(replacement);

        assertThrows(SensorFailureException.class, () -> gearBox.changeTo(GearBox.Gear.N));
        assertEquals(1, replacement.readCount());
        assertEquals(GearBox.Gear.R, gearBox.getCurrentGear());
    }

    private static Stream<Arguments> speedsStrictlyBelowReverseEntryLimit() {
        return Stream.of(
                Arguments.of(STOPPED_SPEED_KMH),
                Arguments.of(Double.MIN_VALUE),
                Arguments.of(Math.nextDown(REVERSE_ENTRY_SPEED_LIMIT_KMH))
        );
    }

    private static Stream<Arguments> exactZeroTransitions() {
        return Stream.of(
                Arguments.of(GearBox.Gear.R, GearBox.Gear.D),
                Arguments.of(GearBox.Gear.D, GearBox.Gear.R)
        );
    }

    private static void assertRejected(GearBox.Gear from, GearBox.Gear to, double speedKmh) {
        GearBox gearBox = gearboxIn(from, sensor(speedKmh));

        assertThrows(IllegalGearChangeException.class, () -> gearBox.changeTo(to));

        assertEquals(from, gearBox.getCurrentGear());
    }

    private static void assertSpeedRejected(double speedKmh) {
        GearBox gearBox = new GearBox(sensor(speedKmh));

        assertThrows(IllegalArgumentException.class, () -> gearBox.changeTo(GearBox.Gear.R));

        assertEquals(GearBox.Gear.P, gearBox.getCurrentGear());
    }

    private static GearBox gearboxIn(GearBox.Gear gear, ConfigurableSpeedSensor sensor) {
        double speedAfterSetup = sensor.speedKmh;
        sensor.setSpeedKmh(STOPPED_SPEED_KMH);
        GearBox gearBox = new GearBox(sensor);
        if (gear != GearBox.Gear.P) {
            gearBox.changeTo(GearBox.Gear.R);
            if (gear != GearBox.Gear.R) {
                gearBox.changeTo(GearBox.Gear.N);
                if (gear == GearBox.Gear.D) {
                    gearBox.changeTo(GearBox.Gear.D);
                }
            }
        }
        sensor.setSpeedKmh(speedAfterSetup);
        return gearBox;
    }

    private static ConfigurableSpeedSensor sensor(double speedKmh) {
        return new ConfigurableSpeedSensor(speedKmh);
    }

    private static final class ConfigurableSpeedSensor implements SpeedSensor {
        private double speedKmh;
        private RuntimeException failure;
        private int readCount;

        private ConfigurableSpeedSensor(double speedKmh) {
            this.speedKmh = speedKmh;
        }

        private int readCount() {
            return readCount;
        }

        private void setSpeedKmh(double speedKmh) {
            this.speedKmh = speedKmh;
            this.failure = null;
        }

        private void failWith(RuntimeException failure) {
            this.failure = failure;
        }

        @Override
        public double getSpeedKmh() {
            readCount++;
            if (failure != null) {
                throw failure;
            }
            return speedKmh;
        }
    }
}
