package frc.robot;

import static org.junit.jupiter.api.Assertions.*;
import java.util.Map;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.align.AlignController;
import frc.robot.align.AlignStateMachine;
import frc.robot.vision.HeadingResetDetector;
import frc.robot.vision.VisionBootstrap;

class HeadingResetDetectorTest {
    private final HeadingResetDetector detector = new HeadingResetDetector(
        Constants.VisionConstants.HEADING_RESET_THRESHOLD_DEG);
    private final VisionBootstrap trust = new VisionBootstrap();

    private void sample(double pose, double raw, boolean bootstrap) {
        if (detector.update(pose, raw, bootstrap)) trust.clearPoseTrust();
    }
    private AlignController controller() {
        return new AlignController(() -> Optional.of(Alliance.Blue), trust::isPoseTrusted);
    }
    private void start(AlignController controller, double heading) {
        controller.start(0, new Pose2d(0, 0, Rotation2d.fromDegrees(heading)),
            new ChassisSpeeds(), Map.of(4, new Pose2d()), new int[]{4});
    }

    @Test void seedAtNinetyDegreesRefusesNextAlignAndLaterBootstrapRestoresTrust() {
        trust.didReset();
        sample(90, 90, false); // First loop only baselines.
        assertTrue(trust.isPoseTrusted());
        sample(0, 90, false); // Blue operator perspective seed: no raw gyro change.
        assertFalse(trust.isPoseTrusted());
        assertEquals(1, trust.getTrustDrops());
        sample(0, 90, false);
        assertEquals(1, trust.getTrustDrops());
        var controller = controller(); start(controller, 0);
        assertEquals(AlignStateMachine.Status.POSE_UNTRUSTED, controller.status());
        assertTrue(trust.shouldReset("camera", false, new int[]{4,8}, new int[]{4,8},
            1, .1, new Pose2d()));
        trust.didReset(); sample(0, 90, true);
        start(controller, 0);
        assertEquals(AlignStateMachine.Status.RUNNING, controller.status());
        assertEquals(1, trust.getTrustDrops());
    }

    @Test void seedDuringRunningAlignEndsUntrustedWithZeroOutput() {
        trust.didReset(); sample(90, 90, false);
        var controller = controller(); start(controller, 90);
        assertFalse(controller.isFinished());
        sample(0, 90, false);
        assertFalse(controller.isAligned(.02));
        var output = controller.update(.02, new Pose2d(), id -> .02);
        assertEquals(AlignStateMachine.Status.POSE_UNTRUSTED, controller.status());
        assertTrue(controller.isFinished());
        assertEquals(0, output.vxMetersPerSecond);
        assertEquals(0, output.vyMetersPerSecond);
        assertEquals(0, output.omegaRadiansPerSecond);
    }

    @Test void smallDriftAndPhysicalTurnKeepTrust() {
        trust.didReset(); sample(0, 0, false);
        sample(94.9, 90, false);
        assertTrue(trust.isPoseTrusted());
        sample(100, 90.1, false); // Exactly five degrees of offset change is allowed.
        assertTrue(trust.isPoseTrusted());
        assertEquals(0, trust.getTrustDrops());
    }

    @Test void bootstrapHeadingChangeIsSkippedAndRebaselined() {
        trust.didReset(); sample(10, 20, false);
        trust.didReset(); sample(90, 20, true);
        assertTrue(trust.isPoseTrusted());
        sample(91, 21, false);
        assertTrue(trust.isPoseTrusted());
        assertEquals(0, trust.getTrustDrops());
        sample(0, 21, false);
        assertFalse(trust.isPoseTrusted());
    }

    @Test void independentWrapCrossingsInBothDirectionsDoNotTrigger() {
        trust.didReset(); sample(179, 169, false);
        sample(-179, 171, false);
        sample(-169, -179, false);
        sample(-179, 171, false);
        sample(179, 169, false);
        assertTrue(trust.isPoseTrusted());
        assertEquals(0, trust.getTrustDrops());
    }
}
