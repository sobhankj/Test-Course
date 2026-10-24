package autopilot.core;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class SteeringControllerTest {

    private static final double LOW_SPEED_KMH = 100.0;
    private static final double HIGH_SPEED_THRESHOLD_KMH = 120.0;
    private static final double ABOVE_HIGH_SPEED_THRESHOLD_KMH = 120.1;
    private static final double MAX_CHANGE_PER_UPDATE_DEGREES = 60.0;
    private static final double HIGH_SPEED_LIMIT_DEGREES = 90.0;
    private static final double PHYSICAL_LIMIT_DEGREES = 450.0;
    private static final double ANGLE_TOLERANCE = 0.0;

    @ParameterizedTest
    @ValueSource(doubles = {0.0, 30.0, -45.0, 60.0, -60.0})
    void update_requestedAngleWithinOneStep_reachesRequestedAngle(double requestedAngleDegrees) {
        SteeringController controller = new SteeringController();

        double actualAngle = controller.update(requestedAngleDegrees, LOW_SPEED_KMH);

        assertEquals(requestedAngleDegrees, actualAngle, ANGLE_TOLERANCE);
    }

    @Test
    void update_largePositiveAngle_movesAtMostSixtyDegreesPerUpdate() {
        SteeringController controller = new SteeringController();

        double firstAngle = controller.update(PHYSICAL_LIMIT_DEGREES, LOW_SPEED_KMH);
        double secondAngle = controller.update(PHYSICAL_LIMIT_DEGREES, LOW_SPEED_KMH);

        assertEquals(MAX_CHANGE_PER_UPDATE_DEGREES, firstAngle, ANGLE_TOLERANCE);
        assertEquals(2 * MAX_CHANGE_PER_UPDATE_DEGREES, secondAngle, ANGLE_TOLERANCE);
    }

    @Test
    void update_largeNegativeAngle_movesAtMostSixtyDegreesPerUpdate() {
        SteeringController controller = new SteeringController();

        double actualAngle = controller.update(-PHYSICAL_LIMIT_DEGREES, LOW_SPEED_KMH);

        assertEquals(-MAX_CHANGE_PER_UPDATE_DEGREES, actualAngle, ANGLE_TOLERANCE);
    }

    @Test
    void update_speedExactlyAtHighSpeedThreshold_continuesPastReducedLimit() {
        SteeringController controller = new SteeringController();

        controller.update(PHYSICAL_LIMIT_DEGREES, HIGH_SPEED_THRESHOLD_KMH);
        controller.update(PHYSICAL_LIMIT_DEGREES, HIGH_SPEED_THRESHOLD_KMH);
        double actualAngle = controller.update(PHYSICAL_LIMIT_DEGREES, HIGH_SPEED_THRESHOLD_KMH);

        assertEquals(3 * MAX_CHANGE_PER_UPDATE_DEGREES, actualAngle, ANGLE_TOLERANCE);
    }

    @Test
    void update_speedAboveHighSpeedThreshold_stopsAtNinetyDegrees() {
        SteeringController controller = new SteeringController();

        controller.update(PHYSICAL_LIMIT_DEGREES, ABOVE_HIGH_SPEED_THRESHOLD_KMH);
        double secondAngle = controller.update(PHYSICAL_LIMIT_DEGREES, ABOVE_HIGH_SPEED_THRESHOLD_KMH);
        double thirdAngle = controller.update(PHYSICAL_LIMIT_DEGREES, ABOVE_HIGH_SPEED_THRESHOLD_KMH);

        assertEquals(HIGH_SPEED_LIMIT_DEGREES, secondAngle, ANGLE_TOLERANCE);
        assertEquals(HIGH_SPEED_LIMIT_DEGREES, thirdAngle, ANGLE_TOLERANCE);
    }

    @Test
    void update_negativeRequestAboveHighSpeed_stopsAtMinusNinetyDegrees() {
        SteeringController controller = new SteeringController();

        controller.update(-PHYSICAL_LIMIT_DEGREES, ABOVE_HIGH_SPEED_THRESHOLD_KMH);
        double actualAngle = controller.update(-PHYSICAL_LIMIT_DEGREES, ABOVE_HIGH_SPEED_THRESHOLD_KMH);

        assertEquals(-HIGH_SPEED_LIMIT_DEGREES, actualAngle, ANGLE_TOLERANCE);
    }

    @Test
    void selfCenter_fromAngleBeyondOneStep_movesTowardZeroBySixtyDegrees() {
        SteeringController controller = new SteeringController();
        controller.update(PHYSICAL_LIMIT_DEGREES, LOW_SPEED_KMH);
        controller.update(PHYSICAL_LIMIT_DEGREES, LOW_SPEED_KMH);

        double firstCenter = controller.selfCenter();
        double secondCenter = controller.selfCenter();

        assertEquals(MAX_CHANGE_PER_UPDATE_DEGREES, firstCenter, ANGLE_TOLERANCE);
        assertEquals(0.0, secondCenter, ANGLE_TOLERANCE);
    }

    @Test
    void update_nonFiniteAngle_throwsIllegalArgumentExceptionAndKeepsAngle() {
        SteeringController controller = new SteeringController();
        controller.update(MAX_CHANGE_PER_UPDATE_DEGREES, LOW_SPEED_KMH);

        assertThrows(IllegalArgumentException.class,
                () -> controller.update(Double.NaN, LOW_SPEED_KMH));
        assertThrows(IllegalArgumentException.class,
                () -> controller.update(Double.POSITIVE_INFINITY, LOW_SPEED_KMH));
        assertEquals(MAX_CHANGE_PER_UPDATE_DEGREES, controller.update(MAX_CHANGE_PER_UPDATE_DEGREES, LOW_SPEED_KMH),
                ANGLE_TOLERANCE);
    }

    @Test
    void update_negativeOrNonFiniteSpeed_throwsIllegalArgumentException() {
        SteeringController controller = new SteeringController();

        assertThrows(IllegalArgumentException.class,
                () -> controller.update(MAX_CHANGE_PER_UPDATE_DEGREES, -0.1));
        assertThrows(IllegalArgumentException.class,
                () -> controller.update(MAX_CHANGE_PER_UPDATE_DEGREES, Double.NaN));
        assertEquals(MAX_CHANGE_PER_UPDATE_DEGREES,
                controller.update(MAX_CHANGE_PER_UPDATE_DEGREES, LOW_SPEED_KMH),
                ANGLE_TOLERANCE);
    }
}
