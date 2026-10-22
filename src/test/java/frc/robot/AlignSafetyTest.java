package frc.robot;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import java.util.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.align.*;
import frc.robot.vision.VisionFilter;

class AlignSafetyTest {
    @Test void unknownAllianceNeverMoves() {
        var controller = new AlignController(Optional::empty);
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
        var controller = new AlignController(() -> Optional.of(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue));
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
        var handoff = edu.wpi.first.wpilibj2.command.Commands.run(() -> zeros[0]++, drive)
            .until(() -> DriverHandoff.sticksNeutral(sticks[0], sticks[1], sticks[2]));
        assertTrue(handoff.getRequirements().contains(drive));
        handoff.initialize();
        for (int i = 0; i < 20; i++) {
            handoff.execute();
            assertFalse(handoff.isFinished());
        }
        sticks[0] = 0;
        handoff.execute();
        assertTrue(handoff.isFinished());
        assertEquals(21, zeros[0]);
        handoff.end(false);
        edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().unregisterSubsystem(drive);
    }
}
