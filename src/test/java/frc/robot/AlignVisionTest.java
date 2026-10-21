package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Map;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.align.AlignMath;
import frc.robot.align.AlignStateMachine;
import frc.robot.align.AlignStateMachine.Status;
import frc.robot.vision.VisionFilter;
import frc.robot.vision.VisionFilter.Result;

class AlignVisionTest {
    // ---- target pose math ----
    @Test
    void targetInFrontOfTagFacingIt() {
        Pose2d tag = new Pose2d(10, 4, Rotation2d.fromDegrees(180)); // normal points -x
        Pose2d t = AlignMath.targetPose(tag, 0.6, 0);
        assertEquals(9.4, t.getX(), 1e-9);
        assertEquals(4.0, t.getY(), 1e-9);
        assertEquals(0.0, t.getRotation().getDegrees(), 1e-9); // facing +x toward tag
    }

    @Test
    void lateralOffsetIsLeftOfViewer() {
        Pose2d tag = new Pose2d(10, 4, Rotation2d.fromDegrees(180));
        Pose2d t = AlignMath.targetPose(tag, 0.6, 0.2);
        // viewer faces +x, their left is +y
        assertEquals(4.2, t.getY(), 1e-9);
    }

    @Test
    void nearestTagSelection() {
        Map<Integer, Pose2d> tags = Map.of(
            1, new Pose2d(0, 0, Rotation2d.kZero),
            2, new Pose2d(5, 0, Rotation2d.kZero),
            3, new Pose2d(1, 0, Rotation2d.kZero));
        var r = new Pose2d(4, 0, Rotation2d.kZero);
        assertEquals(2, AlignMath.nearestTag(r, new int[] {1, 2, 3}, tags).getAsInt());
        assertEquals(3, AlignMath.nearestTag(r, new int[] {1, 3, 99}, tags).getAsInt());
        assertTrue(AlignMath.nearestTag(r, new int[] {}, tags).isEmpty());
        assertTrue(AlignMath.nearestTag(r, new int[] {99}, tags).isEmpty());
    }

    @Test
    void atTargetTolerances() {
        Pose2d t = new Pose2d(1, 1, Rotation2d.kZero);
        assertTrue(AlignMath.atTarget(new Pose2d(1.02, 1, Rotation2d.fromDegrees(1)), t, 0.03, Math.toRadians(2)));
        assertFalse(AlignMath.atTarget(new Pose2d(1.1, 1, Rotation2d.kZero), t, 0.03, Math.toRadians(2)));
        assertFalse(AlignMath.atTarget(new Pose2d(1, 1, Rotation2d.fromDegrees(5)), t, 0.03, Math.toRadians(2)));
        // wraparound
        assertTrue(AlignMath.atTarget(new Pose2d(1, 1, Rotation2d.fromDegrees(359)),
            t, 0.03, Math.toRadians(2)));
    }

    // ---- state machine ----
    @Test
    void alignedOnlyAfterNConsecutiveLoops() {
        var m = new AlignStateMachine(5, 0.5, 3);
        m.start(0);
        m.update(0.02, true, 0.01);
        m.update(0.04, true, 0.03);
        assertFalse(m.isAligned(0.04));
        m.update(0.06, false, 0.05); // breaks streak
        m.update(0.08, true, 0.07);
        m.update(0.10, true, 0.09);
        assertFalse(m.isAligned(0.10));
        m.update(0.12, true, 0.11);
        assertTrue(m.isAligned(0.12));
    }

    @Test
    void alignedFalseWhenVisionGoesStaleEvenBeforeUpdate() {
        var m = new AlignStateMachine(5, 0.5, 1);
        m.start(0);
        m.update(0.1, true, 0.1);
        assertTrue(m.isAligned(0.2));
        assertFalse(m.isAligned(0.7));
    }

    @Test
    void timeoutEndsNotAligned() {
        var m = new AlignStateMachine(2, 0.5, 2);
        m.start(0);
        assertEquals(Status.RUNNING, m.update(1.9, false, 1.9));
        assertEquals(Status.TIMED_OUT, m.update(2.0, true, 2.0));
        assertFalse(m.isAligned(2.0));
    }

    @Test
    void timeoutAfterAlignedStillClearsAligned() {
        var m = new AlignStateMachine(1, 0.5, 1);
        m.start(0);
        m.update(0.5, true, 0.5);
        assertTrue(m.isAligned(0.5));
        assertEquals(Status.TIMED_OUT, m.update(1.0, true, 1.0));
        assertFalse(m.isAligned(1.0));
    }

    @Test
    void lostTargetEndsNotAligned() {
        var m = new AlignStateMachine(10, 0.5, 1);
        m.start(0);
        m.update(0.1, true, 0.1);
        assertTrue(m.isAligned(0.1));
        assertEquals(Status.LOST_TARGET, m.update(0.7, true, 0.1));
        assertFalse(m.isAligned(0.7));
        // stays failed even if vision returns
        assertEquals(Status.LOST_TARGET, m.update(0.8, true, 0.8));
    }

    @Test
    void noVisionEverIsLostImmediately() {
        var m = new AlignStateMachine(10, 0.5, 1);
        m.start(100);
        assertEquals(Status.LOST_TARGET, m.update(100.02, true, Double.NEGATIVE_INFINITY));
    }

    @Test
    void resetAndRestartClearAligned() {
        var m = new AlignStateMachine(10, 0.5, 1);
        m.start(0);
        m.update(0.1, true, 0.1);
        m.reset();
        assertFalse(m.isAligned(0.1));
        m.start(1);
        assertFalse(m.isAligned(1.0));
        assertEquals(Status.RUNNING, m.getStatus());
    }

    // ---- vision filter ----
    @Test
    void visionRejections() {
        double now = 10;
        assertEquals(Result.ACCEPT, VisionFilter.evaluate(2, 10, 9.9, now, 9.0, 720, 0.5));
        assertEquals(Result.NO_TAGS, VisionFilter.evaluate(0, 10, 9.9, now, 9.0, 720, 0.5));
        assertEquals(Result.SPINNING, VisionFilter.evaluate(2, -800, 9.9, now, 9.0, 720, 0.5));
        assertEquals(Result.STALE, VisionFilter.evaluate(2, 10, 9.0, now, 8.0, 720, 0.5));
        assertEquals(Result.DUPLICATE, VisionFilter.evaluate(2, 10, 9.9, now, 9.9, 720, 0.5));
        assertEquals(Result.INVALID, VisionFilter.evaluate(2, 10, Double.NaN, now, 9.0, 720, 0.5));
    }

    @Test
    void stdDevGrowsWithDistance() {
        assertEquals(0.5, VisionFilter.xyStdDev(0, 0.5, 4), 1e-12);
        assertEquals(0.5 * (1 + 16.0 / 4), VisionFilter.xyStdDev(4, 0.5, 4), 1e-12);
        assertTrue(VisionFilter.xyStdDev(3, 0.5, 4) > VisionFilter.xyStdDev(1, 0.5, 4));
        double[] sd = VisionFilter.stdDevs(2);
        assertEquals(sd[0], sd[1]);
        assertTrue(sd[2] > 1e6);
    }
}
