package autopilot.core;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class SteeringControllerTest {

    private static final double LOW_SPEED_KMH = 100.0;
    private static final double HIGH_SPEED_THRESHOLD_KMH = 120.0;
    private static final double ABOVE_HIGH_SPEED_THRESHOLD_KMH = 120.1;
    private static final double MAX_CHANGE_PER_UPDATE_DEGREES = 60.0;
    private static final double HIGH_SPEED_LIMIT_DEGREES = 90.0;
    private static final double PHYSICAL_LIMIT_DEGREES = 450.0;
    private static final double ANGLE_TOLERANCE = 0.0;

    // update -> moveTowardTarget: when the request is already within 60 degrees of zero,
    // the angle jumps straight to that request in one tick.
    @ParameterizedTest
    @ValueSource(doubles = {0.0, 30.0, -45.0, 60.0, -60.0})
    void update_requestedAngleWithinOneStep_reachesRequestedAngle(double requestedAngleDegrees) {
        SteeringController controller = new SteeringController();

        double actualAngle = controller.update(requestedAngleDegrees, LOW_SPEED_KMH);

        assertEquals(requestedAngleDegrees, actualAngle, ANGLE_TOLERANCE);
    }

    // update -> moveTowardTarget: a +450 request is not reached at once.
    // Each call adds at most +60 degrees, so two calls reach +120.
    @Test
    void update_largePositiveAngle_movesAtMostSixtyDegreesPerUpdate() {
        SteeringController controller = new SteeringController();

        double firstAngle = controller.update(PHYSICAL_LIMIT_DEGREES, LOW_SPEED_KMH);
        double secondAngle = controller.update(PHYSICAL_LIMIT_DEGREES, LOW_SPEED_KMH);

        assertEquals(MAX_CHANGE_PER_UPDATE_DEGREES, firstAngle, ANGLE_TOLERANCE);
        assertEquals(2 * MAX_CHANGE_PER_UPDATE_DEGREES, secondAngle, ANGLE_TOLERANCE);
    }

    // update -> moveTowardTarget: the same 60-degree step applies in the negative direction.
    // A -450 request from zero produces -60 on the first call.
    @Test
    void update_largeNegativeAngle_movesAtMostSixtyDegreesPerUpdate() {
        SteeringController controller = new SteeringController();

        double actualAngle = controller.update(-PHYSICAL_LIMIT_DEGREES, LOW_SPEED_KMH);

        assertEquals(-MAX_CHANGE_PER_UPDATE_DEGREES, actualAngle, ANGLE_TOLERANCE);
    }

    // update: the reduced +/-90 limit uses a strict greater-than check on 120 km/h.
    // At exactly 120 the physical +/-450 limit still applies, so three steps reach 180.
    @Test
    void update_speedExactlyAtHighSpeedThreshold_continuesPastReducedLimit() {
        SteeringController controller = new SteeringController();

        controller.update(PHYSICAL_LIMIT_DEGREES, HIGH_SPEED_THRESHOLD_KMH);
        controller.update(PHYSICAL_LIMIT_DEGREES, HIGH_SPEED_THRESHOLD_KMH);
        double actualAngle = controller.update(PHYSICAL_LIMIT_DEGREES, HIGH_SPEED_THRESHOLD_KMH);

        assertEquals(3 * MAX_CHANGE_PER_UPDATE_DEGREES, actualAngle, ANGLE_TOLERANCE);
    }

    // update -> clamp: above 120 km/h the target is clamped to +90.
    // Further updates stay at +90 instead of continuing toward 450.
    @Test
    void update_speedAboveHighSpeedThreshold_stopsAtNinetyDegrees() {
        SteeringController controller = new SteeringController();

        controller.update(PHYSICAL_LIMIT_DEGREES, ABOVE_HIGH_SPEED_THRESHOLD_KMH);
        double secondAngle = controller.update(PHYSICAL_LIMIT_DEGREES, ABOVE_HIGH_SPEED_THRESHOLD_KMH);
        double thirdAngle = controller.update(PHYSICAL_LIMIT_DEGREES, ABOVE_HIGH_SPEED_THRESHOLD_KMH);

        assertEquals(HIGH_SPEED_LIMIT_DEGREES, secondAngle, ANGLE_TOLERANCE);
        assertEquals(HIGH_SPEED_LIMIT_DEGREES, thirdAngle, ANGLE_TOLERANCE);
    }

    // update -> clamp: above 120 km/h a large negative request is clamped to -90
    // and the wheel stops there after the 60-degree steps.
    @Test
    void update_negativeRequestAboveHighSpeed_stopsAtMinusNinetyDegrees() {
        SteeringController controller = new SteeringController();

        controller.update(-PHYSICAL_LIMIT_DEGREES, ABOVE_HIGH_SPEED_THRESHOLD_KMH);
        double actualAngle = controller.update(-PHYSICAL_LIMIT_DEGREES, ABOVE_HIGH_SPEED_THRESHOLD_KMH);

        assertEquals(-HIGH_SPEED_LIMIT_DEGREES, actualAngle, ANGLE_TOLERANCE);
    }

    // update -> clamp: once speed crosses 120 km/h, a wheel already past +90 steps back
    // toward the newly reduced target by 60 degrees per call and then stays there.
    @Test
    void update_speedIncreasesAboveThreshold_movesCurrentAngleTowardReducedTarget() {
        SteeringController controller = new SteeringController();
        controller.update(PHYSICAL_LIMIT_DEGREES, HIGH_SPEED_THRESHOLD_KMH);
        controller.update(PHYSICAL_LIMIT_DEGREES, HIGH_SPEED_THRESHOLD_KMH);
        controller.update(PHYSICAL_LIMIT_DEGREES, HIGH_SPEED_THRESHOLD_KMH);

        controller.update(PHYSICAL_LIMIT_DEGREES, ABOVE_HIGH_SPEED_THRESHOLD_KMH);

        assertEquals(2 * MAX_CHANGE_PER_UPDATE_DEGREES, controller.getCurrentAngleDegrees(), ANGLE_TOLERANCE);
        assertEquals(HIGH_SPEED_LIMIT_DEGREES, controller.getTargetAngleDegrees(), ANGLE_TOLERANCE);

        controller.update(PHYSICAL_LIMIT_DEGREES, ABOVE_HIGH_SPEED_THRESHOLD_KMH);

        assertEquals(HIGH_SPEED_LIMIT_DEGREES, controller.getCurrentAngleDegrees(), ANGLE_TOLERANCE);
        assertEquals(HIGH_SPEED_LIMIT_DEGREES, controller.getTargetAngleDegrees(), ANGLE_TOLERANCE);

        controller.update(PHYSICAL_LIMIT_DEGREES, ABOVE_HIGH_SPEED_THRESHOLD_KMH);

        assertEquals(HIGH_SPEED_LIMIT_DEGREES, controller.getCurrentAngleDegrees(), ANGLE_TOLERANCE);
        assertEquals(HIGH_SPEED_LIMIT_DEGREES, controller.getTargetAngleDegrees(), ANGLE_TOLERANCE);
    }

    // selfCenter -> moveTowardTarget: the target becomes 0, but the wheel still
    // moves at most 60 degrees per call, from 120 to 60 and then to 0.
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

    // update -> validateAngle: NaN and infinite requests are rejected before the angle changes.
    // A later valid update still starts from the previous 60-degree position.
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

    // update -> validateSpeed: a negative or non-finite speed is rejected before any movement.
    // The next valid update still behaves as if the wheel is at zero.
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

    // update -> clamp: below the high-speed threshold the target is limited to +/-450.
    // Values already inside that range, including the limits themselves, are stored unchanged.
    @ParameterizedTest
    @CsvSource({
            "600.0, 450.0",
            "-600.0, -450.0",
            "450.0, 450.0",
            "-450.0, -450.0",
            "200.0, 200.0"
    })
    void update_lowSpeed_clampsTargetToPhysicalLimit(double requestedAngleDegrees, double expectedTargetDegrees) {
        SteeringController controller = new SteeringController();

        controller.update(requestedAngleDegrees, LOW_SPEED_KMH);

        assertEquals(expectedTargetDegrees, controller.getTargetAngleDegrees(), ANGLE_TOLERANCE);
    }

    // update -> clamp: strictly above 120 km/h the target is limited to +/-90.
    // Requests already at +/-90 stay at those limits.
    @ParameterizedTest
    @CsvSource({
            "600.0, 90.0",
            "-600.0, -90.0",
            "90.0, 90.0",
            "-90.0, -90.0"
    })
    void update_aboveHighSpeedThreshold_clampsTargetToHighSpeedLimit(double requestedAngleDegrees,
                                                                      double expectedTargetDegrees) {
        SteeringController controller = new SteeringController();

        controller.update(requestedAngleDegrees, ABOVE_HIGH_SPEED_THRESHOLD_KMH);

        assertEquals(expectedTargetDegrees, controller.getTargetAngleDegrees(), ANGLE_TOLERANCE);
    }

    // update -> clamp: 120 km/h still uses the physical limit. Only a speed strictly above 120
    // reduces a +450 request to +90.
    @ParameterizedTest
    @CsvSource({
            "119.9, 450.0",
            "120.0, 450.0",
            "120.1, 90.0"
    })
    void update_physicalLimitRequest_usesReducedLimitOnlyAboveThreshold(double speedKmh,
                                                                         double expectedTargetDegrees) {
        SteeringController controller = new SteeringController();

        controller.update(PHYSICAL_LIMIT_DEGREES, speedKmh);

        assertEquals(expectedTargetDegrees, controller.getTargetAngleDegrees(), ANGLE_TOLERANCE);
    }

    // update -> validateSpeed: zero is a finite non-negative speed, so the request is accepted.
    @Test
    void update_zeroSpeed_acceptsRequestedAngle() {
        SteeringController controller = new SteeringController();

        double actualAngle = controller.update(30.0, 0.0);

        assertEquals(30.0, actualAngle, ANGLE_TOLERANCE);
        assertEquals(30.0, controller.getCurrentAngleDegrees(), ANGLE_TOLERANCE);
        assertEquals(30.0, controller.getTargetAngleDegrees(), ANGLE_TOLERANCE);
    }

    // Both angles start at zero before any command.
    @Test
    void getters_beforeAnyUpdate_returnZero() {
        SteeringController controller = new SteeringController();

        assertEquals(0.0, controller.getCurrentAngleDegrees(), ANGLE_TOLERANCE);
        assertEquals(0.0, controller.getTargetAngleDegrees(), ANGLE_TOLERANCE);
    }

    // update: a request past the physical limit stores the clamped target immediately,
    // while the current angle advances by only one 60-degree step.
    @Test
    void getters_afterLargeRequest_exposeClampedTargetAndSteppedCurrentAngle() {
        SteeringController controller = new SteeringController();

        double returnedAngle = controller.update(600.0, LOW_SPEED_KMH);

        assertEquals(PHYSICAL_LIMIT_DEGREES, controller.getTargetAngleDegrees(), ANGLE_TOLERANCE);
        assertEquals(MAX_CHANGE_PER_UPDATE_DEGREES, controller.getCurrentAngleDegrees(), ANGLE_TOLERANCE);
        assertEquals(controller.getCurrentAngleDegrees(), returnedAngle, ANGLE_TOLERANCE);
    }

    // selfCenter: the target becomes zero on the call, but the current angle only steps toward it.
    @Test
    void getters_afterSelfCenter_targetIsZeroWhileCurrentAngleStepsTowardZero() {
        SteeringController controller = controllerAt(2 * MAX_CHANGE_PER_UPDATE_DEGREES, PHYSICAL_LIMIT_DEGREES);

        double returnedAngle = controller.selfCenter();

        assertEquals(0.0, controller.getTargetAngleDegrees(), ANGLE_TOLERANCE);
        assertEquals(MAX_CHANGE_PER_UPDATE_DEGREES, controller.getCurrentAngleDegrees(), ANGLE_TOLERANCE);
        assertEquals(controller.getCurrentAngleDegrees(), returnedAngle, ANGLE_TOLERANCE);
    }

    // Reading either getter must leave both stored angles unchanged.
    @Test
    void getters_readingState_doesNotChangeAngles() {
        SteeringController controller = new SteeringController();
        controller.update(PHYSICAL_LIMIT_DEGREES, LOW_SPEED_KMH);
        double currentBefore = controller.getCurrentAngleDegrees();
        double targetBefore = controller.getTargetAngleDegrees();

        controller.getCurrentAngleDegrees();
        controller.getTargetAngleDegrees();

        assertEquals(currentBefore, controller.getCurrentAngleDegrees(), ANGLE_TOLERANCE);
        assertEquals(targetBefore, controller.getTargetAngleDegrees(), ANGLE_TOLERANCE);
    }

    // update -> validateAngle: a non-finite angle is rejected and both stored angles stay put.
    // The controller is left with a current angle different from its target so a reset to zero
    // would be visible.
    @ParameterizedTest
    @ValueSource(doubles = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void update_nonFiniteAngle_preservesCurrentAndTarget(double invalidAngleDegrees) {
        SteeringController controller = controllerAt(MAX_CHANGE_PER_UPDATE_DEGREES, PHYSICAL_LIMIT_DEGREES);
        double currentBefore = controller.getCurrentAngleDegrees();
        double targetBefore = controller.getTargetAngleDegrees();

        assertThrows(IllegalArgumentException.class,
                () -> controller.update(invalidAngleDegrees, LOW_SPEED_KMH));

        assertEquals(currentBefore, controller.getCurrentAngleDegrees(), ANGLE_TOLERANCE);
        assertEquals(targetBefore, controller.getTargetAngleDegrees(), ANGLE_TOLERANCE);
    }

    // update -> validateSpeed: negative and non-finite speeds are rejected without moving either angle.
    @ParameterizedTest
    @ValueSource(doubles = {-0.1, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void update_invalidSpeed_preservesCurrentAndTarget(double invalidSpeedKmh) {
        SteeringController controller = controllerAt(MAX_CHANGE_PER_UPDATE_DEGREES, PHYSICAL_LIMIT_DEGREES);
        double currentBefore = controller.getCurrentAngleDegrees();
        double targetBefore = controller.getTargetAngleDegrees();

        assertThrows(IllegalArgumentException.class,
                () -> controller.update(PHYSICAL_LIMIT_DEGREES, invalidSpeedKmh));

        assertEquals(currentBefore, controller.getCurrentAngleDegrees(), ANGLE_TOLERANCE);
        assertEquals(targetBefore, controller.getTargetAngleDegrees(), ANGLE_TOLERANCE);
    }

    // selfCenter -> moveTowardTarget: a new controller is already centered, so the call stays at zero.
    @Test
    void selfCenter_onNewController_returnsZero() {
        SteeringController controller = new SteeringController();

        double actualAngle = controller.selfCenter();

        assertEquals(0.0, actualAngle, ANGLE_TOLERANCE);
        assertEquals(0.0, controller.getCurrentAngleDegrees(), ANGLE_TOLERANCE);
        assertEquals(0.0, controller.getTargetAngleDegrees(), ANGLE_TOLERANCE);
    }

    // selfCenter -> moveTowardTarget: a remaining gap of exactly +60 degrees is closed in one call.
    @Test
    void selfCenter_fromExactlySixtyDegrees_reachesZeroInOneCall() {
        SteeringController controller = new SteeringController();
        controller.update(MAX_CHANGE_PER_UPDATE_DEGREES, LOW_SPEED_KMH);

        double actualAngle = controller.selfCenter();

        assertEquals(0.0, actualAngle, ANGLE_TOLERANCE);
        assertEquals(0.0, controller.getCurrentAngleDegrees(), ANGLE_TOLERANCE);
        assertEquals(0.0, controller.getTargetAngleDegrees(), ANGLE_TOLERANCE);
    }

    // selfCenter -> moveTowardTarget: a remaining gap of exactly -60 degrees is closed in one call.
    @Test
    void selfCenter_fromExactlyMinusSixtyDegrees_reachesZeroInOneCall() {
        SteeringController controller = new SteeringController();
        controller.update(-MAX_CHANGE_PER_UPDATE_DEGREES, LOW_SPEED_KMH);

        double actualAngle = controller.selfCenter();

        assertEquals(0.0, actualAngle, ANGLE_TOLERANCE);
        assertEquals(0.0, controller.getCurrentAngleDegrees(), ANGLE_TOLERANCE);
        assertEquals(0.0, controller.getTargetAngleDegrees(), ANGLE_TOLERANCE);
    }

    // selfCenter -> moveTowardTarget: from -120 the first call moves +60 to -60, then the next reaches zero.
    @Test
    void selfCenter_fromNegativeAngleBeyondOneStep_movesTowardZeroBySixtyDegrees() {
        SteeringController controller = controllerAt(-2 * MAX_CHANGE_PER_UPDATE_DEGREES, -PHYSICAL_LIMIT_DEGREES);

        double firstCenter = controller.selfCenter();
        double secondCenter = controller.selfCenter();

        assertEquals(-MAX_CHANGE_PER_UPDATE_DEGREES, firstCenter, ANGLE_TOLERANCE);
        assertEquals(0.0, controller.getTargetAngleDegrees(), ANGLE_TOLERANCE);
        assertEquals(0.0, secondCenter, ANGLE_TOLERANCE);
    }

    // selfCenter -> moveTowardTarget: repeated calls from the physical limit walk to zero
    // in 60-degree steps and stay there.
    @Test
    void selfCenter_repeatedCallsFromPhysicalLimit_reachesZeroWithoutOvershoot() {
        SteeringController controller = controllerAt(PHYSICAL_LIMIT_DEGREES, PHYSICAL_LIMIT_DEGREES);

        double previous = controller.getCurrentAngleDegrees();
        for (int step = 0; step < 7; step++) {
            double angle = controller.selfCenter();
            assertEquals(previous - MAX_CHANGE_PER_UPDATE_DEGREES, angle, ANGLE_TOLERANCE);
            assertTrue(angle > 0.0);
            assertEquals(0.0, controller.getTargetAngleDegrees(), ANGLE_TOLERANCE);
            previous = angle;
        }

        assertEquals(30.0, previous, ANGLE_TOLERANCE);
        assertEquals(0.0, controller.selfCenter(), ANGLE_TOLERANCE);
        assertEquals(0.0, controller.selfCenter(), ANGLE_TOLERANCE);
        assertEquals(0.0, controller.getCurrentAngleDegrees(), ANGLE_TOLERANCE);
    }

    // update -> moveTowardTarget: after sitting at +60, a -90 request reverses direction.
    // The first step is exactly 60 degrees and stops short of the new target.
    @Test
    void update_fromPositiveToNegativeTarget_movesTowardNewTargetWithoutOvershoot() {
        SteeringController controller = new SteeringController();
        controller.update(MAX_CHANGE_PER_UPDATE_DEGREES, LOW_SPEED_KMH);

        double actualAngle = controller.update(-HIGH_SPEED_LIMIT_DEGREES, LOW_SPEED_KMH);

        assertEquals(0.0, actualAngle, ANGLE_TOLERANCE);
        assertEquals(0.0, controller.getCurrentAngleDegrees(), ANGLE_TOLERANCE);
        assertEquals(-HIGH_SPEED_LIMIT_DEGREES, controller.getTargetAngleDegrees(), ANGLE_TOLERANCE);
        assertTrue(actualAngle > controller.getTargetAngleDegrees());
    }

    // update -> moveTowardTarget: after sitting at -60, a +90 request reverses direction
    // by exactly 60 degrees and does not cross the new target.
    @Test
    void update_fromNegativeToPositiveTarget_movesTowardNewTargetWithoutOvershoot() {
        SteeringController controller = new SteeringController();
        controller.update(-MAX_CHANGE_PER_UPDATE_DEGREES, LOW_SPEED_KMH);

        double actualAngle = controller.update(HIGH_SPEED_LIMIT_DEGREES, LOW_SPEED_KMH);

        assertEquals(0.0, actualAngle, ANGLE_TOLERANCE);
        assertEquals(0.0, controller.getCurrentAngleDegrees(), ANGLE_TOLERANCE);
        assertEquals(HIGH_SPEED_LIMIT_DEGREES, controller.getTargetAngleDegrees(), ANGLE_TOLERANCE);
        assertTrue(actualAngle < controller.getTargetAngleDegrees());
    }

    // update -> moveTowardTarget: from +60, the remaining gap can be exactly 60, under 60, or over 60.
    // The current angle reaches the target in one call only when the gap is at most 60.
    @ParameterizedTest
    @CsvSource({
            "120.0, 120.0, 120.0",
            "90.0, 90.0, 90.0",
            "200.0, 120.0, 200.0"
    })
    void update_fromSixtyDegrees_movesAtMostSixtyDegreesTowardRequest(double requestedAngleDegrees,
                                                                       double expectedCurrentDegrees,
                                                                       double expectedTargetDegrees) {
        SteeringController controller = new SteeringController();
        controller.update(MAX_CHANGE_PER_UPDATE_DEGREES, LOW_SPEED_KMH);

        double actualAngle = controller.update(requestedAngleDegrees, LOW_SPEED_KMH);

        assertEquals(expectedCurrentDegrees, actualAngle, ANGLE_TOLERANCE);
        assertEquals(expectedCurrentDegrees, controller.getCurrentAngleDegrees(), ANGLE_TOLERANCE);
        assertEquals(expectedTargetDegrees, controller.getTargetAngleDegrees(), ANGLE_TOLERANCE);
    }

    // Reaches a low-speed state through update(). When the angles differ, the current angle is
    // staged one 60-degree step before the target, including when the two angles have opposite signs.
    private static SteeringController controllerAt(double currentAngleDegrees, double targetAngleDegrees) {
        if (!isReachableLowSpeedAngle(currentAngleDegrees) || !isReachableLowSpeedAngle(targetAngleDegrees)) {
            throw new IllegalArgumentException("steering state is outside the public low-speed range");
        }

        SteeringController controller = new SteeringController();
        if (currentAngleDegrees == targetAngleDegrees) {
            settleAt(controller, currentAngleDegrees);
        } else {
            double direction = Math.signum(targetAngleDegrees - currentAngleDegrees);
            double stagingAngle = currentAngleDegrees - direction * MAX_CHANGE_PER_UPDATE_DEGREES;
            if (!isReachableLowSpeedAngle(stagingAngle)) {
                throw new IllegalArgumentException("steering state is not reachable in one movement step");
            }
            settleAt(controller, stagingAngle);
            controller.update(targetAngleDegrees, LOW_SPEED_KMH);
        }

        if (controller.getCurrentAngleDegrees() != currentAngleDegrees
                || controller.getTargetAngleDegrees() != targetAngleDegrees) {
            throw new IllegalStateException("test setup did not reach the requested steering state");
        }
        return controller;
    }

    private static void settleAt(SteeringController controller, double angleDegrees) {
        int steps = (int) Math.ceil(Math.abs(angleDegrees) / MAX_CHANGE_PER_UPDATE_DEGREES);
        for (int step = 0; step < steps; step++) {
            controller.update(angleDegrees, LOW_SPEED_KMH);
        }
    }

    private static boolean isReachableLowSpeedAngle(double angleDegrees) {
        return Double.isFinite(angleDegrees) && Math.abs(angleDegrees) <= PHYSICAL_LIMIT_DEGREES;
    }
}
