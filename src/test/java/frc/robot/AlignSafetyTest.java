package frc.robot;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import java.util.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.align.*;
import frc.robot.vision.VisionFilter;

class AlignSafetyTest {
    private static java.util.function.BooleanSupplier trustedPose() {
        var policy = new frc.robot.vision.VisionBootstrap();
        if (!policy.shouldReset(false, new int[]{4,8}, new int[]{4,8}, 1, .1,
                new Pose2d())) throw new AssertionError("bootstrap rejected");
        policy.didReset();
        return policy::isPoseTrusted;
    }

    @Test void unknownAllianceNeverMoves() {
        var controller = new AlignController(Optional::empty, trustedPose());
        controller.start(0, new Pose2d(), new ChassisSpeeds(),
            Map.of(4, new Pose2d(3, 3, new Rotation2d())), new int[]{4});
        for (int i = 0; i < 250; i++) {
            var output = controller.update(i*.02, new Pose2d(), id -> 0);
            assertEquals(0, Math.hypot(output.vxMetersPerSecond, output.vyMetersPerSecond));
            assertEquals(0, output.omegaRadiansPerSecond);
            assertFalse(controller.isAligned(i*.02));
        }
        assertTrue(controller.isFinished());
    }

    @Test void diagonalCapPreservesDirection() {
        var controller = new AlignController(() -> Optional.of(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue), trustedPose());
        // Target is (3,3), so equal errors must produce equal components.
        controller.start(0, new Pose2d(), new ChassisSpeeds(),
            Map.of(4, new Pose2d(2.4, 3, new Rotation2d())), new int[]{4});
        for (int i = 0; i < 250; i++) {
            final double now = i*.02;
            var output = controller.update(now, new Pose2d(), id -> now);
            assertTrue(Math.hypot(output.vxMetersPerSecond, output.vyMetersPerSecond) <= Constants.AlignConstants.MAX_SPEED_MPS + 1e-9);
            assertEquals(output.vxMetersPerSecond, output.vyMetersPerSecond, 1e-9);
        }
    }

    @Test void bootstrapAndSubsequentJumpGate() {
        var measured = new Pose2d();
        var estimate = new Pose2d(3, 0, new Rotation2d());
        int[] layout = {4,8};
        assertFalse(VisionFilter.shouldBootstrap(false, new int[]{4}, layout));
        assertEquals(VisionFilter.Result.POSE_JUMP, VisionFilter.evaluatePoseJump(measured, estimate, false));
        assertFalse(VisionFilter.shouldBootstrap(false, new int[]{4,999}, layout));
        assertFalse(VisionFilter.shouldBootstrap(false, new int[]{4,4}, layout));
        assertTrue(VisionFilter.shouldBootstrap(false, layout, layout));
        assertEquals(VisionFilter.Result.ACCEPT, VisionFilter.evaluatePoseJump(measured, estimate, true));
        estimate = new Pose2d(measured.getTranslation(), estimate.getRotation()); // production resetPose policy
        assertFalse(VisionFilter.shouldBootstrap(true, layout, layout));
        assertEquals(VisionFilter.Result.ACCEPT, VisionFilter.evaluatePoseJump(new Pose2d(.5,0,new Rotation2d()), estimate, false));
        assertEquals(VisionFilter.Result.POSE_JUMP, VisionFilter.evaluatePoseJump(new Pose2d(.501,0,new Rotation2d()), estimate, false));
    }

    @Test void bootstrapEligibleDuringAlignUsesNormalGateWithoutReset() {
        int[] layout = {4, 8};
        var estimate = new Pose2d(3, 0, new Rotation2d());
        var measurement = new Pose2d();
        assertTrue(VisionFilter.shouldBootstrap(false, false, layout, layout));
        boolean bootstrap = VisionFilter.shouldBootstrap(false, true, layout, layout);
        assertFalse(bootstrap); // Production resetPose branch is reachable only with bootstrap=true.
        assertEquals(VisionFilter.Result.POSE_JUMP,
            VisionFilter.evaluatePoseJump(measurement, estimate, bootstrap));
        assertEquals(VisionFilter.Result.ACCEPT,
            VisionFilter.evaluatePoseJump(estimate, estimate, bootstrap));
        assertFalse(VisionFilter.shouldBootstrap(false, false, new int[]{4,999}, layout));
    }

    @Test void forcedPoseResetLatchesZeroAndClearsAlignment() {
        for (boolean headingOnly : new boolean[]{false, true}) {
            var controller = new AlignController(() -> Optional.of(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue), trustedPose());
            var tag = new Pose2d(3, 3, new Rotation2d());
            var target = AlignMath.targetPose(tag, Constants.AlignConstants.OFFSET_DISTANCE_M, 0);
            controller.start(0, target, new ChassisSpeeds(), Map.of(4, tag), new int[]{4});
            for (int i=0; i<10; i++) {
                final double now = i*.02;
                controller.update(now, target, id -> now);
            }
            assertTrue(controller.isAligned(.18));
            var reset = headingOnly
                ? new Pose2d(target.getTranslation(), target.getRotation().plus(Rotation2d.fromDegrees(90)))
                : new Pose2d(target.getX()+2, target.getY(), target.getRotation());
            for (int i=10; i<30; i++) {
                final double now = i*.02;
                var output = controller.update(now, reset, id -> now);
                assertEquals(0, output.vxMetersPerSecond);
                assertEquals(0, output.vyMetersPerSecond);
                assertEquals(0, output.omegaRadiansPerSecond);
                assertTrue(controller.isFinished());
                assertFalse(controller.isAligned(now));
                assertEquals(AlignStateMachine.Status.POSE_RESET, controller.status());
                assertEquals(target, controller.target());
            }
        }
    }

    @Test void bootstrapDuringAlignmentIncludingHeldAtEnableUsesNormalGate() {
        // bootstrapped=false models the enable edge, including a trigger already held.
        for (boolean bootstrapped : new boolean[]{false, true}) {
            for (int[] ids : new int[][]{ {4,8}, {4} }) {
                boolean reset = VisionFilter.shouldBootstrap(bootstrapped, true, ids, new int[]{4,8}, 1, .1);
                assertFalse(reset);
                assertEquals(VisionFilter.Result.POSE_JUMP, VisionFilter.evaluatePoseJump(
                    new Pose2d(3,0,new Rotation2d()), new Pose2d(), reset));
                assertEquals(VisionFilter.Result.ACCEPT, VisionFilter.evaluatePoseJump(
                    new Pose2d(.1,0,new Rotation2d()), new Pose2d(), reset));
            }
        }
    }

    @Test void singleTagBootstrapRequiresCloseConfidentLayoutTag() {
        int[] ids = {4}, layout = {4,8};
        assertTrue(VisionFilter.shouldBootstrap(false, false, ids, layout, 1.99, .19));
        assertFalse(VisionFilter.shouldBootstrap(false, false, ids, layout, 2, .1));
        assertFalse(VisionFilter.shouldBootstrap(false, false, ids, layout, 1, .2));
        assertFalse(VisionFilter.shouldBootstrap(false, false, ids, layout, Double.NaN, .1));
        assertFalse(VisionFilter.shouldBootstrap(false, false, ids, layout, 1, Double.NaN));
        assertFalse(VisionFilter.shouldBootstrap(false, false, new int[]{999}, layout, 1, .1));
        assertFalse(VisionFilter.shouldBootstrap(true, false, ids, layout, 1, .1));
        assertTrue(VisionFilter.shouldBootstrap(false, false, layout, layout, Double.NaN, Double.NaN));
    }

    @Test void forcedTranslationOrHeadingResetLatchesZeroAndNotAligned() {
        for (Pose2d reset : new Pose2d[]{new Pose2d(3,0,new Rotation2d()),
                new Pose2d(0,0,Rotation2d.fromDegrees(90))}) {
            var controller = new AlignController(() -> Optional.of(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue), trustedPose());
            controller.start(0, new Pose2d(), new ChassisSpeeds(),
                Map.of(4, new Pose2d(3,0,new Rotation2d())), new int[]{4});
            controller.update(.02, new Pose2d(), id -> .02);
            for (int i=2; i<10; i++) {
                var out = controller.update(i*.02, reset, id -> .04);
                assertEquals(0, out.vxMetersPerSecond);
                assertEquals(0, out.vyMetersPerSecond);
                assertEquals(0, out.omegaRadiansPerSecond);
                assertTrue(controller.isFinished());
                assertFalse(controller.isAligned(i*.02));
                assertEquals(AlignStateMachine.Status.POSE_RESET, controller.status());
            }
        }
    }

    @Test void releaseHoldsUntilAllSticksInsideManualDeadband() {
        assertFalse(DriverHandoff.sticksNeutral(.2, 0, 0));
        assertFalse(DriverHandoff.sticksNeutral(.08, .08, 0));
        assertFalse(DriverHandoff.sticksNeutral(0, 0, .1));
        assertFalse(DriverHandoff.sticksNeutral(Double.NaN, 0, 0));
        assertTrue(DriverHandoff.sticksNeutral(.03, .04, .09));
        assertTrue(DriverHandoff.sticksNeutral(0, 0, 0));
    }
    @Test void releaseCommandRetainsDriveRequirementUntilNeutral() {
        edu.wpi.first.hal.HAL.initialize(500, 0);
        var drive = new edu.wpi.first.wpilibj2.command.SubsystemBase() {};
        double[] sticks = {.3, 0, 0};
        int[] zeros = {0};
        var handoff = DriverHandoff.holdZero(() -> zeros[0]++,
            () -> DriverHandoff.sticksNeutral(sticks[0], sticks[1], sticks[2]), drive);
        assertTrue(handoff.getRequirements().contains(drive));
        handoff.initialize();
        for (int i = 0; i < 20; i++) {
            handoff.execute();
            assertFalse(handoff.isFinished());
            assertTrue(edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getBoolean("Align/HandoffHoldingZero", false));
            assertEquals("Release sticks to drive", edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getString("Align/HandoffReason", ""));
        }
        sticks[0] = 0;
        handoff.execute();
        assertTrue(handoff.isFinished());
        assertEquals(22, zeros[0]);
        handoff.end(false);
        assertFalse(edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getBoolean("Align/HandoffHoldingZero", true));
        assertEquals("", edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getString("Align/HandoffReason", "missing"));
        edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().unregisterSubsystem(drive);
    }
}
