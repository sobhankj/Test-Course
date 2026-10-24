package autopilot.core;

import autopilot.exception.SensorFailureException;
import autopilot.sensor.LaneSensor;
import autopilot.sensor.ObstacleSensor;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class AutoDriverTest {

    private static final double SPEED_KMH = 80.0;
    private static final double TARGET_SPEED_KMH = 80.0;
    private static final double CLEAR_DISTANCE_METERS = 100.0;
    private static final double CENTERED_LANE_OFFSET_METERS = 0.0;
    private static final double POSITIVE_LANE_OFFSET_METERS = 1.0;
    private static final double NEGATIVE_LANE_OFFSET_METERS = -1.0;
    private static final double LANE_CORRECTION_DEGREES = 45.0;
    private static final double SAFE_MODE_BRAKE_PRESSURE = 30.0;
    private static final double VALUE_TOLERANCE = 1.0e-6;

    @Test
    void tick_speedAtTargetAndLaneCentered_holdsWithoutBrakingOrSteering() {
        AutoDriver driver = driver(SPEED_KMH, CLEAR_DISTANCE_METERS, CENTERED_LANE_OFFSET_METERS);

        AutoDriver.DriveResult result = driver.tick(GearBox.Gear.D);

        assertFalse(result.safeMode());
        assertFalse(result.warningActive());
        assertEquals(AdaptiveCruise.Mode.HOLD, result.cruiseMode());
        assertEquals(0.0, result.brakePressure(), VALUE_TOLERANCE);
        assertEquals(0.0, result.steeringAngleDegrees(), VALUE_TOLERANCE);
    }

    @Test
    void tick_positiveLaneOffset_steersLeftByCorrectionAngle() {
        AutoDriver driver = driver(SPEED_KMH, CLEAR_DISTANCE_METERS, POSITIVE_LANE_OFFSET_METERS);

        AutoDriver.DriveResult result = driver.tick(GearBox.Gear.D);

        assertEquals(-LANE_CORRECTION_DEGREES, result.steeringAngleDegrees(), VALUE_TOLERANCE);
        assertFalse(result.safeMode());
    }

    @Test
    void tick_negativeLaneOffset_steersRightByCorrectionAngle() {
        AutoDriver driver = driver(SPEED_KMH, CLEAR_DISTANCE_METERS, NEGATIVE_LANE_OFFSET_METERS);

        AutoDriver.DriveResult result = driver.tick(GearBox.Gear.D);

        assertEquals(LANE_CORRECTION_DEGREES, result.steeringAngleDegrees(), VALUE_TOLERANCE);
        assertFalse(result.safeMode());
    }

    @Test
    void tick_speedSensorFailure_entersSafeMode() {
        SpeedSensor failedSpeedSensor = () -> {
            throw new SensorFailureException("Speed sensor failed");
        };
        AutoDriver driver = new AutoDriver(
                failedSpeedSensor,
                () -> CLEAR_DISTANCE_METERS,
                () -> CENTERED_LANE_OFFSET_METERS,
                TARGET_SPEED_KMH);

        AutoDriver.DriveResult result = driver.tick(GearBox.Gear.D);

        assertSafeMode(result);
    }

    @Test
    void tick_obstacleSensorFailure_entersSafeMode() {
        ObstacleSensor failedObstacleSensor = () -> {
            throw new SensorFailureException("Obstacle sensor failed");
        };
        AutoDriver driver = new AutoDriver(
                () -> SPEED_KMH,
                failedObstacleSensor,
                () -> CENTERED_LANE_OFFSET_METERS,
                TARGET_SPEED_KMH);

        AutoDriver.DriveResult result = driver.tick(GearBox.Gear.D);

        assertSafeMode(result);
    }

    private static void assertSafeMode(AutoDriver.DriveResult result) {
        assertTrue(result.safeMode());
        assertFalse(result.warningActive());
        assertEquals(AdaptiveCruise.Mode.BRAKE, result.cruiseMode());
        assertEquals(SAFE_MODE_BRAKE_PRESSURE, result.brakePressure(), VALUE_TOLERANCE);
        assertEquals(0.0, result.steeringAngleDegrees(), VALUE_TOLERANCE);
    }

    private static AutoDriver driver(double speedKmh, double distanceMeters, double laneOffsetMeters) {
        return new AutoDriver(
                () -> speedKmh,
                () -> distanceMeters,
                () -> laneOffsetMeters,
                TARGET_SPEED_KMH);
    }
}
