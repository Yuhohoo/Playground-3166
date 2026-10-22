package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.align.FieldLayoutLoader;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Fuses Limelight MegaTag2 poses into the drivetrain estimator. Inert unless FUSION_ENABLED. */
public class VisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final List<String> cameras = new ArrayList<>();
    private final List<Double> lastPerCamera = new ArrayList<>();
    private final TagFreshness tagFreshness = new TagFreshness();
    private java.util.function.BooleanSupplier alignActive = () -> false;
    private final VisionBootstrap bootstrapPolicy;

    private double lastAcceptedTimestamp = Double.NEGATIVE_INFINITY;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this(drivetrain, edu.wpi.first.wpilibj.DriverStation::isEnabled);
    }

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain, java.util.function.BooleanSupplier enabled) {
        this.bootstrapPolicy = new VisionBootstrap(enabled);
        this.drivetrain = drivetrain;
        for (String name : new String[] {VisionConstants.CAMERA_1, VisionConstants.CAMERA_2}) {
            if (name != null && !name.isEmpty()) {
                cameras.add(name);
                lastPerCamera.add(Double.NEGATIVE_INFINITY);
            }
        }
    }

    /** Injected from the align command scheduling state; defaults inactive when no align binding exists. */
    public void setAlignActiveSupplier(java.util.function.BooleanSupplier alignActive) {
        this.alignActive = java.util.Objects.requireNonNull(alignActive);
    }

    public boolean isPoseTrusted() { return bootstrapPolicy.isPoseTrusted(); }

    /** FPGA timestamp (s) of the newest accepted frame from any camera; -inf if none. */
    public double getLastAcceptedTimestamp() {
        return lastAcceptedTimestamp;
    }

    /** FPGA timestamp (s) of the newest accepted frame that contained {@code tagId}; -inf if none. */
    public double getLastAcceptedTimestampForTag(int tagId) {
        return tagFreshness.get(tagId);
    }

    @Override
    public void periodic() {
        boolean ready = bootstrapPolicy.beginLoop();
        bootstrapPolicy.observeAlign(alignActive.getAsBoolean());
        SmartDashboard.putBoolean("Align/PoseTrusted", isPoseTrusted());
        // Subsystem periodic precedes trigger polling: defer enable edge for held align triggers.
        if (!ready) return;
        if (!VisionConstants.FUSION_ENABLED || !FieldLayoutLoader.isAvailable()) return;
        var state = drivetrain.getState();
        double headingDeg = state.Pose.getRotation().getDegrees();
        double yawRateDegPerSec = Math.toDegrees(state.Speeds.omegaRadiansPerSecond);
        double now = Timer.getFPGATimestamp();

        for (int i = 0; i < cameras.size(); i++) {
            String cam = cameras.get(i);
            LimelightHelpers.SetRobotOrientation(cam, headingDeg, yawRateDegPerSec, 0, 0, 0, 0);
            var est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cam);
            if (est == null) { bootstrapPolicy.rejectFrame(); continue; }
            // Validate the entire frame against the layout before fusing its pose.
            int[] ids = est.rawFiducials == null ? new int[0]
                : java.util.Arrays.stream(est.rawFiducials).mapToInt(f -> f.id).toArray();
            int[] allowed = VisionFilter.filterAllowed(ids, AlignConstants.PEG_TAG_IDS);
            int[] layoutIds = FieldLayoutLoader.getDeployLayout().orElseThrow()
                .getTags().stream().mapToInt(t -> t.ID).toArray();
            var result = VisionFilter.evaluateIds(ids, layoutIds);
            if (result == VisionFilter.Result.ACCEPT) result = VisionFilter.evaluate(ids.length, yawRateDegPerSec, est.timestampSeconds,
                now, lastPerCamera.get(i));
            if (result != VisionFilter.Result.ACCEPT) { bootstrapPolicy.rejectFrame(); continue; }
            // Consume each exposure once, including bootstrap candidates outside the jump gate.
            lastPerCamera.set(i, est.timestampSeconds);
            boolean bootstrap = bootstrapPolicy.shouldReset(alignActive.getAsBoolean(), ids, layoutIds,
                ids.length == 1 ? est.rawFiducials[0].distToCamera : Double.NaN,
                ids.length == 1 ? est.rawFiducials[0].ambiguity : Double.NaN, est.pose);
            if (!bootstrap && bootstrapPolicy.isCollectingSingleTag()) {
                SmartDashboard.putString("Vision/" + cam + "/Result", "STABILIZING");
                continue;
            }
            if (result == VisionFilter.Result.ACCEPT)
                result = VisionFilter.evaluatePoseJump(est.pose, drivetrain.getState().Pose, bootstrap);
            SmartDashboard.putString("Vision/" + cam + "/Result", result.name());
            if (result != VisionFilter.Result.ACCEPT) { bootstrapPolicy.rejectFrame(); continue; }

            double[] sd = VisionFilter.stdDevs(est.avgTagDist);
            // Drivetrain override applies Utils.fpgaToCurrentTime; pass the raw FPGA timestamp.
            if (bootstrap) {
                // Initialize translation, retaining the authoritative gyro heading.
                drivetrain.resetPose(new edu.wpi.first.math.geometry.Pose2d(
                    est.pose.getTranslation(), drivetrain.getState().Pose.getRotation()));
                bootstrapPolicy.didReset();
            } else {
                drivetrain.addVisionMeasurement(est.pose, est.timestampSeconds,
                    VecBuilder.fill(sd[0], sd[1], sd[2]));
            }
            lastPerCamera.set(i, est.timestampSeconds);
            tagFreshness.record(allowed, est.timestampSeconds);
            lastAcceptedTimestamp = Math.max(lastAcceptedTimestamp, est.timestampSeconds);
        }
        SmartDashboard.putBoolean("Align/PoseTrusted", isPoseTrusted());
        SmartDashboard.putNumber("Vision/LastAcceptedAgeSec", now - lastAcceptedTimestamp);
    }
}
