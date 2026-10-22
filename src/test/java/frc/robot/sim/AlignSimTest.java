package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.*;
import java.nio.file.*;
import java.util.*;
import org.junit.jupiter.api.Test;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.AlignConstants;
import frc.robot.align.*;
import frc.robot.vision.*;
import frc.robot.commands.AlignGating;
import edu.wpi.first.wpilibj2.command.Commands;

class AlignSimTest {
    private static java.util.function.BooleanSupplier trustedPose() {
        var policy = new frc.robot.vision.VisionBootstrap();
        if (!policy.shouldReset(false, new int[]{4,8}, new int[]{4,8}, 1, .1,
                new Pose2d())) throw new AssertionError("bootstrap rejected");
        policy.didReset();
        return policy::isPoseTrusted;
    }

    private final Map<Integer, Pose2d> tags = new HashMap<>();
    private final int[] layoutIds;
    AlignSimTest() throws Exception {
        var layout = new AprilTagFieldLayout(Path.of("src/test/resources/placeholder_tags.json"));
        layout.getTags().forEach(t -> tags.put(t.ID, t.pose.toPose2d()));
        layoutIds = layout.getTags().stream().mapToInt(t -> t.ID).toArray();
    }
    record Frame(double captured, double delivery, Pose2d pose, int[] ids) {}
    record Scenario(String name, double distance, double side, double heading, String event, double cap) {}
    private void scoreDenied(AlignController controller, double now) {
        int[] shots = {0};
        var command = AlignGating.gatedWhile(Commands.runOnce(() -> shots[0]++),
            () -> controller.isAligned(now), () -> {});
        command.initialize(); command.execute(); command.end(false);
        assertEquals(0, shots[0], "unsafe scoring");
    }
    @Test void diagonalCapPreservesDirection() {
        var controller = new AlignController(() -> Optional.of(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue), trustedPose());
        var target = AlignMath.targetPose(tags.get(4), AlignConstants.OFFSET_DISTANCE_M, 0);
        var pose = new Pose2d(target.getX()-3, target.getY()-3, target.getRotation());
        controller.start(0, pose, new ChassisSpeeds(), tags, new int[]{4});
        boolean reachedCap = false;
        for (int i=0; i<200; i++) {
            double now = i*.02;
            var output = controller.update(now, pose, id -> now);
            double norm = Math.hypot(output.vxMetersPerSecond, output.vyMetersPerSecond);
            assertTrue(norm <= AlignConstants.MAX_SPEED_MPS+1e-9);
            assertEquals(output.vxMetersPerSecond, output.vyMetersPerSecond, 1e-9);
            reachedCap |= norm >= AlignConstants.MAX_SPEED_MPS-1e-9;
        }
        assertTrue(reachedCap);
    }

    @Test void emptyPegSelectionEndsWithoutOutput() {
        var controller = new AlignController(() -> java.util.Optional.of(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue), trustedPose());
        controller.start(0, new Pose2d(), new ChassisSpeeds(), tags, new int[0]);
        assertNull(controller.target()); assertEquals(-1, controller.targetId());
        assertTrue(controller.isFinished()); assertFalse(controller.isAligned(0));
        var out = controller.update(0, new Pose2d(), id -> 0);
        assertEquals(0, out.vxMetersPerSecond); assertEquals(0, out.vyMetersPerSecond);
        assertEquals(0, out.omegaRadiansPerSecond);
        assertEquals(VisionFilter.Result.ACCEPT, VisionFilter.evaluateIds(new int[]{1,4}, layoutIds));
    }
    @Test void scoreAfterAlignmentButLostTargetNeverShoots() {
        edu.wpi.first.hal.HAL.initialize(500, 0);
        var controller = new AlignController(() -> java.util.Optional.of(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue), trustedPose());
        var target = AlignMath.targetPose(tags.get(4), AlignConstants.OFFSET_DISTANCE_M, 0);
        controller.start(0, target, new ChassisSpeeds(), tags, new int[]{4});
        for (int i=0; i<AlignConstants.ALIGNED_LOOPS; i++) {
            final double now = i*.02;
            controller.update(now, target, id -> now);
        }
        assertTrue(controller.isAligned(.18));
        // No timeout or update is needed for score gating to observe expired target freshness.
        assertEquals(AlignStateMachine.Status.RUNNING, controller.status());
        assertFalse(controller.isAligned(.70));
        scoreDenied(controller, .70);
        var output = controller.update(.70, target, id -> .18);
        assertEquals(0, output.vxMetersPerSecond);
        assertEquals(0, output.vyMetersPerSecond);
        assertEquals(0, output.omegaRadiansPerSecond);
        assertEquals(AlignStateMachine.Status.LOST_TARGET, controller.status());
    }

    @Test void closedLoopScenarios() throws Exception {
        edu.wpi.first.hal.HAL.initialize(500, 0);
        var scenarios = List.of(
            new Scenario("near", .65, 0, 0, "none", 1),
            new Scenario("far", 2.7, 0, 0, "none", 1),
            new Scenario("heading 45deg", 1.4, 0, 45, "none", 1),
            new Scenario("FOV edge", 1.4, 0, 49, "none", 1),
            new Scenario("red alliance", 1.5, 1.0, 0, "red", 1),
            new Scenario("untrusted pose", 1.5, 1.0, 0, "untrusted", 1),
            new Scenario("unknown alliance", 1.5, 1.0, 0, "unknown", 1),
            new Scenario("side", 1.5, 1.0, 0, "none", 1),
            new Scenario("lost halfway", 2.7, 0, 0, "lost", 1),
            new Scenario("release/restart", 2.7, 0, 0, "release", 1),
            new Scenario("non-peg + unknown", 1.5, 0, 0, "mixed", 1),
            new Scenario("forced pose reset", 1.5, 0, 0, "reset", 1),
            new Scenario("pose jump", 1.5, 0, 0, "jump", 1),
            new Scenario("timeout", 8, 0, 0, "timeout", 1),
            new Scenario("hypothetical high", .65, 0, 0, "none", .5),
            new Scenario("hypothetical unhomed", .35, 0, 0, "none", .2));
        StringBuilder table = new StringBuilder("| Scenario | Converge s | Final xy m | Theta deg | Overshoot m | Sign changes | False aligned | End | Cap violations |\n|---|---:|---:|---:|---:|---:|---:|---|---:|\n");
        for (var scenario : scenarios) table.append(run(scenario));
        Files.createDirectories(Path.of("build"));
        Files.writeString(Path.of("build/sim-results.md"), table.toString());
        System.out.println("\n"+table);
    }
    String run(Scenario s) throws Exception {
        var random = new Random(3166);
        Pose2d target = AlignMath.targetPose(tags.get(4), AlignConstants.OFFSET_DISTANCE_M, 0);
        var robot = new SimRobot(new Pose2d(target.getX()-s.distance, target.getY()+s.side,
            Rotation2d.fromDegrees(s.heading)));
        assertEquals(10.84, robot.maxSpeed, 1e-6); assertTrue(robot.manualDeadband > 1);
        var controller = new AlignController(() -> s.event.equals("unknown") ? Optional.empty()
            : Optional.of(s.event.equals("red") ? edu.wpi.first.wpilibj.DriverStation.Alliance.Red
                : edu.wpi.first.wpilibj.DriverStation.Alliance.Blue),
            s.event.equals("untrusted") ? () -> false : trustedPose());
        controller.start(0, robot.pose, robot.speed, tags, AlignConstants.PEG_TAG_IDS);
        scoreDenied(controller, 0);
        var freshness = new TagFreshness();
        var pending = new ArrayDeque<Frame>();
        Pose2d estimated = robot.pose;
        double converge = Double.NaN, overshoot = 0, previousError = target.getX()-robot.pose.getX();
        int signs = 0, falseAligned = 0, violations = 0;
        boolean bootstrapped = false, jumpRejected = false;
        boolean released = false, restarted = false, sawAligned = false, unknownRejected = false;
        double stoppedAt = Double.NaN;
        for (int i=0; i<=250; i++) {
            double now = i*.02;
            boolean lost = s.event.equals("lost") && now >= 1;
            // Camera +x faces robot forward, horizontal half FOV 50 degrees, no range clipping.
            double bearing = Math.atan2(tags.get(4).getY()-robot.pose.getY(), tags.get(4).getX()-robot.pose.getX());
            boolean visible = Math.abs(new Rotation2d(bearing).minus(robot.pose.getRotation()).getDegrees()) <= 50;
            if (!lost && visible && random.nextDouble() >= .08) {
                double latency = .06;
                int[] ids = s.event.equals("mixed") ? (i==25 ? new int[]{4,999} : new int[]{4,1}) : new int[]{4};
                double jump = s.event.equals("jump") && i==40 ? .65 : 0;
                var noisy = new Pose2d(robot.pose.getX()+random.nextGaussian()*.002+jump,
                    robot.pose.getY()+random.nextGaussian()*.002,
                    new Rotation2d(robot.pose.getRotation().getRadians()+random.nextGaussian()*.001));
                pending.add(new Frame(now, now+latency, noisy, ids));
            }
            while (!pending.isEmpty() && pending.peek().delivery <= now+1e-9) {
                var frame = pending.remove();
                var valid = VisionFilter.evaluateIds(frame.ids, layoutIds);
                boolean bootstrap = VisionFilter.shouldBootstrap(bootstrapped, !controller.isFinished(), frame.ids, layoutIds);
                if (valid == VisionFilter.Result.ACCEPT) {
                    valid = VisionFilter.evaluatePoseJump(frame.pose, estimated, bootstrap);
                    if (valid == VisionFilter.Result.POSE_JUMP) jumpRejected = true;
                }
                if (valid == VisionFilter.Result.ACCEPT && VisionFilter.evaluate(frame.ids.length, 0,
                        frame.captured, now, Double.NEGATIVE_INFINITY) == VisionFilter.Result.ACCEPT) {
                    bootstrapped |= bootstrap;
                    // Latency compensation with exact model odometry; deliberately not a CTRE estimator claim.
                    estimated = new Pose2d(frame.pose.getX()+robot.speed.vxMetersPerSecond*(now-frame.captured),
                        frame.pose.getY()+robot.speed.vyMetersPerSecond*(now-frame.captured),
                        new Rotation2d(frame.pose.getRotation().getRadians()+robot.speed.omegaRadiansPerSecond*(now-frame.captured)));
                    freshness.record(VisionFilter.filterAllowed(frame.ids, AlignConstants.PEG_TAG_IDS), frame.captured);
                    if (s.event.equals("mixed")) assertTrue(estimated.getTranslation().getDistance(robot.pose.getTranslation()) < .04, "non-peg caused jump");
                } else if (valid == VisionFilter.Result.UNKNOWN_ID) {
                    unknownRejected = true;
                    assertArrayEquals(new int[]{4,999}, frame.ids);
                }
            }
            // Before first delivered frame, use a captured t=0 exposure as initialization freshness.
            // This models scheduling alignment once the camera has an accepted frame.
            if (i==0) freshness.record(new int[]{4}, 0);
            if (s.event.equals("release") && i==40) {
                controller.stop(); released = true;
                var zero = controller.update(now, estimated, freshness::get);
                assertEquals(0, Math.hypot(zero.vxMetersPerSecond, zero.vyMetersPerSecond));
                assertEquals(0, zero.omegaRadiansPerSecond); assertFalse(controller.isAligned(now));
            }
            if (released && !restarted && i==50) {
                controller.start(now, robot.pose, robot.speed, tags, AlignConstants.PEG_TAG_IDS);
                assertFalse(controller.isAligned(now)); restarted = true;
            }
            // External reset changes only estimation, not the physical robot pose.
            if (s.event.equals("reset") && i==40)
                estimated = new Pose2d(estimated.getX()+2, estimated.getY(), estimated.getRotation());
            var output = controller.update(now, estimated, freshness::get);
            if (s.event.equals("reset") && i>=40) {
                assertEquals(AlignStateMachine.Status.POSE_RESET, controller.status());
                assertTrue(controller.isFinished());
                assertFalse(controller.isAligned(now));
                assertEquals(0, output.vxMetersPerSecond);
                assertEquals(0, output.vyMetersPerSecond);
                assertEquals(0, output.omegaRadiansPerSecond);
            }
            if (lost && now-freshness.get(4) > AlignConstants.LOST_TARGET_SEC) {
                assertEquals(0, output.vxMetersPerSecond); assertEquals(0, output.vyMetersPerSecond);
                assertEquals(0, output.omegaRadiansPerSecond);
                if (Double.isNaN(stoppedAt)) stoppedAt = now;
            }
            if (s.event.equals("unknown") || s.event.equals("untrusted")) {
                assertTrue(controller.isFinished()); assertFalse(controller.isAligned(now));
                assertEquals(0, Math.hypot(output.vxMetersPerSecond, output.vyMetersPerSecond));
                assertEquals(0, output.omegaRadiansPerSecond);
            }
            if (Math.hypot(output.vxMetersPerSecond, output.vyMetersPerSecond)>AlignConstants.MAX_SPEED_MPS+1e-9
                    || Math.abs(output.omegaRadiansPerSecond)>Math.PI+1e-9) violations++;
            // Test-only elevator policy: no elevator exists in production. Apply after controller output.
            output.vxMetersPerSecond = Math.max(-s.cap, Math.min(s.cap, output.vxMetersPerSecond));
            output.vyMetersPerSecond = Math.max(-s.cap, Math.min(s.cap, output.vyMetersPerSecond));
            assertTrue(Math.abs(output.vxMetersPerSecond)<=s.cap+1e-9);
            assertTrue(Math.abs(output.vyMetersPerSecond)<=s.cap+1e-9);
            boolean trueAt = AlignMath.atTarget(robot.pose, target, .04, Math.toRadians(2.5));
            if (controller.isAligned(now)) {
                sawAligned = true;
                if (!trueAt) falseAligned++;
                if (Double.isNaN(converge)) converge = now;
            }
            double error = target.getX()-robot.pose.getX();
            if (error*previousError<0 && Math.abs(error)>.001) signs++;
            previousError = error; overshoot = Math.max(overshoot, -error);
            robot.step(output, .02);
            // Propagate odometry between camera updates.
            estimated = new Pose2d(estimated.getX()+robot.speed.vxMetersPerSecond*.02,
                estimated.getY()+robot.speed.vyMetersPerSecond*.02,
                new Rotation2d(estimated.getRotation().getRadians()+robot.speed.omegaRadiansPerSecond*.02));
        }
        assertEquals(0, falseAligned, s.name); assertEquals(0, violations, s.name);
        double xy = robot.pose.getTranslation().getDistance(target.getTranslation());
        double theta = Math.abs(robot.pose.getRotation().minus(target.getRotation()).getDegrees());
        if (s.event.equals("unknown") || s.event.equals("untrusted")) {
            assertFalse(sawAligned);
            assertEquals(Math.hypot(s.distance, s.side), xy, 1e-9);
        } else if (s.event.equals("reset")) {
            assertEquals(AlignStateMachine.Status.POSE_RESET, controller.status());
            assertFalse(sawAligned);
        } else if (s.event.equals("lost")) {
            assertEquals(AlignStateMachine.Status.LOST_TARGET, controller.status());
            assertTrue(stoppedAt <= 1.52); assertFalse(sawAligned);
        } else if (s.event.equals("timeout")) {
            assertEquals(AlignStateMachine.Status.TIMED_OUT, controller.status()); assertFalse(sawAligned);
        } else {
            assertTrue(sawAligned, s.name+" did not converge: "+xy);
            assertTrue(xy<.04, s.name+" xy="+xy); assertTrue(theta<2.5);
            assertTrue(overshoot<.15); assertTrue(signs<15);
        }
        if (s.event.equals("mixed")) assertTrue(unknownRejected);
        if (s.event.equals("jump")) assertTrue(jumpRejected, "injected jump must be rejected");
        // An alignment that was achieved cannot authorize a shot after its target becomes stale.
        if (sawAligned) { assertFalse(controller.isAligned(6)); scoreDenied(controller, 6); }
        return String.format(Locale.ROOT,"| %s | %s | %.3f | %.2f | %.3f | %d | %d | %s | %d |%n",
            s.name, Double.isNaN(converge)?"—":String.format(Locale.ROOT,"%.2f",converge), xy, theta,
            overshoot, signs, falseAligned, s.event.equals("unknown") ? "REFUSED" : controller.status(), violations);
    }
}
