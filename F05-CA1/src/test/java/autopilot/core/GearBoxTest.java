package autopilot.core;

import autopilot.exception.IllegalGearChangeException;
import autopilot.exception.SensorFailureException;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

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

    // changeTo -> isAllowed: P to R uses a strict less-than check, so 5 km/h and above throw
    // IllegalGearChangeException and leave the gear at P.
    @Test
    void changeTo_parkToReverseAtOrAboveFive_throwsAndStaysInPark() {
        assertRejected(GearBox.Gear.P, GearBox.Gear.R, REVERSE_ENTRY_SPEED_LIMIT_KMH);
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

    // changeTo -> isAllowed: P to N, P to D, D to N, D to P, N to R, and R to P are not allowed at any speed.
    // Each rejection throws IllegalGearChangeException and keeps the current gear.
    @Test
    void changeTo_prohibitedTransitions_throwAndKeepCurrentGear() {
        assertRejected(GearBox.Gear.P, GearBox.Gear.N, STOPPED_SPEED_KMH);
        assertRejected(GearBox.Gear.P, GearBox.Gear.D, STOPPED_SPEED_KMH);
        assertRejected(GearBox.Gear.D, GearBox.Gear.N, STOPPED_SPEED_KMH);
        assertRejected(GearBox.Gear.D, GearBox.Gear.P, STOPPED_SPEED_KMH);
        assertRejected(GearBox.Gear.N, GearBox.Gear.R, STOPPED_SPEED_KMH);
        assertRejected(GearBox.Gear.R, GearBox.Gear.P, STOPPED_SPEED_KMH);
    }

    // changeTo early return: requesting the current gear for every Gear value returns before the sensor is read,
    // so a failing sensor does not change the gear.
    @ParameterizedTest
    @EnumSource(GearBox.Gear.class)
    void changeTo_sameGear_keepsGearWithoutReadingSensor(GearBox.Gear gear) {
        ConfigurableSpeedSensor sensor = sensor(STOPPED_SPEED_KMH);
        GearBox gearBox = gearboxIn(gear, sensor);
        sensor.failWith(new SensorFailureException("speed sensor failed"));

        gearBox.changeTo(gear);

        assertEquals(gear, gearBox.getCurrentGear());
    }

    // changeTo -> validateSpeed: a negative, NaN, or infinite sensor speed throws IllegalArgumentException
    // and leaves the gear at P.
    @Test
    void changeTo_negativeOrNonFiniteSpeed_throwsAndKeepsGear() {
        assertSpeedRejected(-0.1);
        assertSpeedRejected(Double.NaN);
        assertSpeedRejected(Double.POSITIVE_INFINITY);
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
        GearBox gearBox = new GearBox(sensor(STOPPED_SPEED_KMH));

        NullPointerException exception = assertThrows(NullPointerException.class, () -> gearBox.changeTo(null));

        assertEquals("targetGear must not be null", exception.getMessage());
        assertEquals(GearBox.Gear.P, gearBox.getCurrentGear());
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

        private ConfigurableSpeedSensor(double speedKmh) {
            this.speedKmh = speedKmh;
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
            if (failure != null) {
                throw failure;
            }
            return speedKmh;
        }
    }
}
