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
    private double lastAcceptedTimestamp = Double.NEGATIVE_INFINITY;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        for (String name : new String[] {VisionConstants.CAMERA_1, VisionConstants.CAMERA_2}) {
            if (name != null && !name.isEmpty()) {
                cameras.add(name);
                lastPerCamera.add(Double.NEGATIVE_INFINITY);
            }
        }
    }

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
        if (!VisionConstants.FUSION_ENABLED || !FieldLayoutLoader.isAvailable()) return;
        var state = drivetrain.getState();
        double headingDeg = state.Pose.getRotation().getDegrees();
        double yawRateDegPerSec = Math.toDegrees(state.Speeds.omegaRadiansPerSecond);
        double now = Timer.getFPGATimestamp();

        for (int i = 0; i < cameras.size(); i++) {
            String cam = cameras.get(i);
            LimelightHelpers.SetRobotOrientation(cam, headingDeg, yawRateDegPerSec, 0, 0, 0, 0);
            var est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cam);
            if (est == null) continue;
            // Only allowed (peg) tag IDs count; frames with only other IDs are NO_TAGS.
            int[] ids = est.rawFiducials == null ? new int[0]
                : java.util.Arrays.stream(est.rawFiducials).mapToInt(f -> f.id).toArray();
            int[] allowed = VisionFilter.filterAllowed(ids, AlignConstants.PEG_TAG_IDS);
            var result = VisionFilter.evaluate(allowed.length, yawRateDegPerSec, est.timestampSeconds,
                now, lastPerCamera.get(i));
            SmartDashboard.putString("Vision/" + cam + "/Result", result.name());
            if (result != VisionFilter.Result.ACCEPT) continue;

            double[] sd = VisionFilter.stdDevs(est.avgTagDist);
            // Drivetrain override applies Utils.fpgaToCurrentTime; pass the raw FPGA timestamp.
            drivetrain.addVisionMeasurement(est.pose, est.timestampSeconds,
                VecBuilder.fill(sd[0], sd[1], sd[2]));
            lastPerCamera.set(i, est.timestampSeconds);
            tagFreshness.record(allowed, est.timestampSeconds);
            lastAcceptedTimestamp = Math.max(lastAcceptedTimestamp, est.timestampSeconds);
        }
        SmartDashboard.putNumber("Vision/LastAcceptedAgeSec", now - lastAcceptedTimestamp);
    }
}
