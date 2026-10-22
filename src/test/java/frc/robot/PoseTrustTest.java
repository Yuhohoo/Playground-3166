package frc.robot;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import java.util.Optional;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.vision.VisionBootstrap;
import frc.robot.align.AlignController;
import frc.robot.align.AlignStateMachine;

class PoseTrustTest {
    private boolean single(VisionBootstrap policy, int id, double x, boolean aligning) {
        return policy.shouldReset("camera-a", aligning, new int[]{id}, new int[]{4,8}, 1, .1,
            new Pose2d(x, 0, new edu.wpi.first.math.geometry.Rotation2d()));
    }

    private boolean cameraSingle(VisionBootstrap policy, String camera) {
        return policy.shouldReset(camera, false, new int[]{4}, new int[]{4,8}, 1, .1,
            new Pose2d());
    }

    @Test void alternatingDuplicatesPreserveFiveFreshSingleTagFrames() {
        var policy = new VisionBootstrap();
        double last = Double.NEGATIVE_INFINITY;
        for (int fresh = 1; fresh <= 5; fresh++) {
            double timestamp = fresh * .02;
            var result = frc.robot.vision.VisionFilter.evaluate(1, 0, timestamp, timestamp, last);
            assertTrue(policy.acceptFrame("camera-a", result));
            last = timestamp;
            assertEquals(fresh == 5, cameraSingle(policy, "camera-a"));
            result = frc.robot.vision.VisionFilter.evaluate(1, 0, timestamp, timestamp + .01, last);
            assertEquals(frc.robot.vision.VisionFilter.Result.DUPLICATE, result);
            assertFalse(policy.acceptFrame("camera-a", result));
            assertTrue(policy.isCollectingSingleTag("camera-a"));
        }
        policy.didReset();
        assertTrue(policy.isPoseTrusted());
    }

    @Test void movedWhileDisabledClearsTrustOnNextVisionJumpAndRefusesAlign() {
        var policy = new VisionBootstrap();
        policy.beginLoop(true, 0); policy.didReset();
        var controller = new AlignController(() -> Optional.of(Alliance.Blue), policy::isPoseTrusted);
        var tags = Map.of(4, new Pose2d());
        var target = frc.robot.align.AlignMath.targetPose(tags.get(4),
            Constants.AlignConstants.OFFSET_DISTANCE_M, Constants.AlignConstants.OFFSET_LATERAL_M);
        controller.start(0, target, new ChassisSpeeds(), tags, new int[]{4});
        for (int i=0; i<Constants.AlignConstants.ALIGNED_LOOPS; i++) {
            double now = i*.02; controller.update(now, target, id -> now);
        }
        assertTrue(controller.isAligned(.18)); controller.stop();
        assertTrue(policy.isPoseTrusted());
        policy.beginLoop(false, 1); policy.beginLoop(true, 4);
        assertTrue(policy.isPoseTrusted());
        assertTrue(policy.beginLoop(true, 4.02));
        var moved = new Pose2d(target.getX()+2, target.getY(), target.getRotation());
        boolean reset = policy.shouldReset("camera-a", false, new int[]{4,8}, new int[]{4,8}, 1, .1, moved);
        assertFalse(reset);
        assertEquals(frc.robot.vision.VisionFilter.Result.POSE_JUMP,
            policy.evaluatePoseJump(moved, target, reset));
        assertFalse(policy.isPoseTrusted());
        controller.start(4.02, target, new ChassisSpeeds(), tags, new int[]{4});
        assertEquals(AlignStateMachine.Status.POSE_UNTRUSTED, controller.status());
        assertFalse(controller.isAligned(4.02));
        var output = controller.update(4.02, target, id -> 4.02);
        assertEquals(0, output.vxMetersPerSecond);
        assertEquals(0, output.vyMetersPerSecond);
        assertEquals(0, output.omegaRadiansPerSecond);
    }

    @Test void emptySecondCameraDoesNotInterruptStableFirstCameraReset() {
        var policy = new VisionBootstrap();
        for (int frame = 1; frame <= 5; frame++) {
            boolean reset = cameraSingle(policy, "camera-a");
            // Same path as a missing estimate or a NO_TAGS rejection in VisionSubsystem.
            policy.rejectFrame("camera-b");
            assertEquals(frame == 5, reset);
            if (reset) policy.didReset();
        }
        assertTrue(policy.isPoseTrusted());
        assertFalse(policy.isCollectingSingleTag("camera-a"));
    }

    @Test void ownRejectedFrameRestartsOnlyThatCamerasWindow() {
        var policy = new VisionBootstrap();
        for (int frame = 0; frame < 4; frame++) {
            assertFalse(cameraSingle(policy, "camera-a"));
            assertFalse(cameraSingle(policy, "camera-b"));
        }
        policy.rejectFrame("camera-a");
        assertFalse(policy.isCollectingSingleTag("camera-a"));
        assertTrue(policy.isCollectingSingleTag("camera-b"));
        assertTrue(cameraSingle(policy, "camera-b"));
        for (int frame = 0; frame < 4; frame++) assertFalse(cameraSingle(policy, "camera-a"));
        assertTrue(cameraSingle(policy, "camera-a"));
    }

    @Test void scheduledAlignClearsEveryCamerasWindow() {
        var policy = new VisionBootstrap();
        for (int frame = 0; frame < 4; frame++) {
            assertFalse(cameraSingle(policy, "camera-a"));
            assertFalse(cameraSingle(policy, "camera-b"));
        }
        policy.observeAlign(true);
        for (int frame = 0; frame < 4; frame++) {
            assertFalse(cameraSingle(policy, "camera-a"));
            assertFalse(cameraSingle(policy, "camera-b"));
        }
        assertTrue(cameraSingle(policy, "camera-a"));
        assertTrue(cameraSingle(policy, "camera-b"));
    }

    @Test void unstableSingleTagCannotResetButStableWindowCan() {
        var policy = new VisionBootstrap();
        for (int i=0; i<20; i++) assertFalse(single(policy, 4, i%2 == 0 ? 0 : .06, false));
        policy.rejectFrame("camera-a");
        for (int i=0; i<4; i++) assertFalse(single(policy, 4, i*.01, false));
        assertTrue(single(policy, 4, .02, false));
        assertFalse(policy.isPoseTrusted()); // Eligibility alone is not a successful reset.
        policy.didReset();
        assertTrue(policy.isPoseTrusted());
    }

    @Test void idRejectionAndScheduledAlignRestartWindow() {
        var policy = new VisionBootstrap();
        for (int i=0; i<4; i++) assertFalse(single(policy, 4, 0, false));
        assertFalse(single(policy, 8, 0, false));
        for (int i=0; i<3; i++) assertFalse(single(policy, 8, 0, false));
        policy.rejectFrame("camera-a");
        for (int i=0; i<4; i++) assertFalse(single(policy, 8, 0, false));
        for (int i=0; i<10; i++) assertFalse(single(policy, 8, 0, true));
        for (int i=0; i<4; i++) assertFalse(single(policy, 8, 0, false));
        assertTrue(single(policy, 8, 0, false));
    }

    @Test void trustSurvivesShortInjectedDisable() {
        var enabled = new AtomicBoolean(false);
        double[] now = {0};
        var policy = new VisionBootstrap(enabled::get, () -> now[0]);
        assertFalse(policy.isPoseTrusted());
        assertFalse(policy.beginLoop());
        enabled.set(true);
        assertFalse(policy.beginLoop());
        assertFalse(policy.isPoseTrusted());
        assertTrue(policy.beginLoop());
        policy.didReset();
        assertTrue(policy.isPoseTrusted());
        enabled.set(false); policy.beginLoop();
        now[0] = 3;
        enabled.set(true); assertFalse(policy.beginLoop());
        assertTrue(policy.isPoseTrusted());
    }

    @Test void longDisableClearsWhileDisabledAndOnReenable() {
        for (boolean pollDisabled : new boolean[]{false, true}) {
            var policy = new VisionBootstrap();
            policy.beginLoop(true, 0); policy.didReset();
            policy.beginLoop(false, 10);
            policy.beginLoop(false, 15); // Strictly longer than the limit.
            assertTrue(policy.isPoseTrusted());
            policy.beginLoop(!pollDisabled, 15.01);
            assertFalse(policy.isPoseTrusted());
            var controller = new AlignController(() -> Optional.of(Alliance.Blue), policy::isPoseTrusted);
            controller.start(16, new Pose2d(), new ChassisSpeeds(), Map.of(4, new Pose2d()), new int[]{4});
            assertEquals(AlignStateMachine.Status.POSE_UNTRUSTED, controller.status());
        }
    }

    @Test void jumpGateRejectionClearsTrustButOrdinaryRejectionDoesNot() {
        var policy = new VisionBootstrap(); policy.didReset();
        policy.rejectFrame("camera-a"); policy.observeAlign(true);
        assertTrue(policy.isPoseTrusted());
        assertEquals(frc.robot.vision.VisionFilter.Result.POSE_JUMP,
            policy.evaluatePoseJump(new Pose2d(2, 0, new edu.wpi.first.math.geometry.Rotation2d()), new Pose2d(), false));
        assertFalse(policy.isPoseTrusted());
    }

    @Test void autoEndsTrustedAndTeleopCanStartImmediatelyAfterShortDisable() {
        var policy = new VisionBootstrap(); policy.beginLoop(true, 0); policy.didReset();
        var controller = new AlignController(() -> Optional.of(Alliance.Blue), policy::isPoseTrusted);
        controller.setPoseResetHandler(policy::clearPoseTrust);
        var tags = Map.of(4, new Pose2d());
        var target = frc.robot.align.AlignMath.targetPose(tags.get(4),
            Constants.AlignConstants.OFFSET_DISTANCE_M, Constants.AlignConstants.OFFSET_LATERAL_M);
        controller.start(0, target, new ChassisSpeeds(), tags, new int[]{4});
        for (int i=0; i<Constants.AlignConstants.ALIGNED_LOOPS; i++) {
            double now = i*.02; controller.update(now, target, id -> now);
        }
        assertTrue(controller.isAligned(.18)); controller.stop();
        assertTrue(policy.isPoseTrusted());
        policy.beginLoop(false, 1); policy.beginLoop(true, 4);
        controller.start(4, target, new ChassisSpeeds(), tags, new int[]{4});
        assertFalse(controller.isFinished());
        controller.update(4.02, new Pose2d(target.getX()+2, target.getY(), target.getRotation()), id -> 4.02);
        assertEquals(AlignStateMachine.Status.POSE_RESET, controller.status());
        controller.stop();
        assertFalse(policy.isPoseTrusted());
        controller.start(4.04, target, new ChassisSpeeds(), tags, new int[]{4});
        assertEquals(AlignStateMachine.Status.POSE_UNTRUSTED, controller.status());
    }

    @Test void alignRefusesUntilSuccessfulBootstrap() {
        var policy = new VisionBootstrap();
        var controller = new AlignController(() -> Optional.of(Alliance.Blue), policy::isPoseTrusted);
        var pose = new Pose2d();
        var tags = Map.of(4, pose);
        controller.start(0, pose, new ChassisSpeeds(), tags, new int[]{4});
        assertTrue(controller.isFinished());
        assertFalse(controller.isAligned(0));
        assertEquals(AlignStateMachine.Status.POSE_UNTRUSTED, controller.status());
        var zero = controller.update(0, pose, id -> 0);
        assertEquals(0, zero.vxMetersPerSecond);
        assertEquals(0, zero.vyMetersPerSecond);
        assertEquals(0, zero.omegaRadiansPerSecond);
        assertTrue(policy.shouldReset("camera-a", false, new int[]{4,8}, new int[]{4,8}, 1, .1, pose));
        policy.didReset();
        controller.start(0, pose, new ChassisSpeeds(), tags, new int[]{4});
        assertFalse(controller.isFinished());
        assertEquals(AlignStateMachine.Status.RUNNING, controller.status());
        var output = controller.update(.02, pose, id -> .02);
        assertTrue(Math.hypot(output.vxMetersPerSecond, output.vyMetersPerSecond) > 0);
    }
}
