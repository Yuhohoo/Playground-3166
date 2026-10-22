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
        return policy.shouldReset(aligning, new int[]{id}, new int[]{4,8}, 1, .1,
            new Pose2d(x, 0, new edu.wpi.first.math.geometry.Rotation2d()));
    }

    @Test void unstableSingleTagCannotResetButStableWindowCan() {
        var policy = new VisionBootstrap();
        for (int i=0; i<20; i++) assertFalse(single(policy, 4, i%2 == 0 ? 0 : .06, false));
        policy.rejectFrame();
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
        policy.rejectFrame();
        for (int i=0; i<4; i++) assertFalse(single(policy, 8, 0, false));
        for (int i=0; i<10; i++) assertFalse(single(policy, 8, 0, true));
        for (int i=0; i<4; i++) assertFalse(single(policy, 8, 0, false));
        assertTrue(single(policy, 8, 0, false));
    }

    @Test void trustResetsOnInjectedEnableEdge() {
        var enabled = new AtomicBoolean(false);
        var policy = new VisionBootstrap(enabled::get);
        assertFalse(policy.isPoseTrusted());
        assertFalse(policy.beginLoop());
        enabled.set(true);
        assertFalse(policy.beginLoop());
        assertFalse(policy.isPoseTrusted());
        assertTrue(policy.beginLoop());
        policy.didReset();
        assertTrue(policy.isPoseTrusted());
        enabled.set(false); policy.beginLoop();
        enabled.set(true); assertFalse(policy.beginLoop());
        assertFalse(policy.isPoseTrusted());
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
        assertTrue(policy.shouldReset(false, new int[]{4,8}, new int[]{4,8}, 1, .1, pose));
        policy.didReset();
        controller.start(0, pose, new ChassisSpeeds(), tags, new int[]{4});
        assertFalse(controller.isFinished());
        assertEquals(AlignStateMachine.Status.RUNNING, controller.status());
        var output = controller.update(.02, pose, id -> .02);
        assertTrue(Math.hypot(output.vxMetersPerSecond, output.vyMetersPerSecond) > 0);
    }
}
