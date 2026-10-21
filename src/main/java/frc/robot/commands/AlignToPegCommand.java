package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;
import java.util.OptionalInt;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignConstants;
import frc.robot.align.AlignMath;
import frc.robot.align.AlignStateMachine;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Hold-to-align to the nearest peg tag. Ends NOT aligned on timeout or lost vision. */
public class AlignToPegCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier lastVisionTimestamp;
    private final Map<Integer, Pose2d> tagPoses = new HashMap<>();

    private final ProfiledPIDController xPID = new ProfiledPIDController(
        AlignConstants.XY_KP, AlignConstants.XY_KI, AlignConstants.XY_KD,
        new TrapezoidProfile.Constraints(AlignConstants.MAX_SPEED_MPS, AlignConstants.MAX_ACCEL_MPS2));
    private final ProfiledPIDController yPID = new ProfiledPIDController(
        AlignConstants.XY_KP, AlignConstants.XY_KI, AlignConstants.XY_KD,
        new TrapezoidProfile.Constraints(AlignConstants.MAX_SPEED_MPS, AlignConstants.MAX_ACCEL_MPS2));
    private final ProfiledPIDController thetaPID = new ProfiledPIDController(
        AlignConstants.THETA_KP, AlignConstants.THETA_KI, AlignConstants.THETA_KD,
        new TrapezoidProfile.Constraints(AlignConstants.MAX_ANGULAR_RAD_PER_SEC,
            AlignConstants.MAX_ANGULAR_ACCEL_RAD_PER_SEC2));

    // Blue-origin field-centric: no operator-perspective flip.
    private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
        .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance);

    private final AlignStateMachine machine = new AlignStateMachine(
        AlignConstants.TIMEOUT_SEC, AlignConstants.LOST_TARGET_SEC, AlignConstants.ALIGNED_LOOPS);

    private Pose2d target;

    public AlignToPegCommand(CommandSwerveDrivetrain drivetrain, DoubleSupplier lastVisionTimestamp) {
        this.drivetrain = drivetrain;
        this.lastVisionTimestamp = lastVisionTimestamp;
        thetaPID.enableContinuousInput(-Math.PI, Math.PI);
        xPID.setTolerance(AlignConstants.XY_TOLERANCE_M);
        yPID.setTolerance(AlignConstants.XY_TOLERANCE_M);
        thetaPID.setTolerance(AlignConstants.THETA_TOLERANCE_RAD);
        if (AlignConstants.ALIGN_ENABLED) {
            try {
                AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AlignConstants.FIELD_LAYOUT);
                for (int id : AlignConstants.PEG_TAG_IDS) {
                    layout.getTagPose(id).ifPresent(p -> tagPoses.put(id, p.toPose2d()));
                }
            } catch (RuntimeException e) {
                System.err.println("AlignToPeg: failed to load field layout: " + e);
            }
        }
        addRequirements(drivetrain);
    }

    public boolean isAligned() {
        return isScheduled() && machine.isAligned(Timer.getFPGATimestamp());
    }

    @Override
    public void initialize() {
        target = null;
        machine.start(Timer.getFPGATimestamp());
        if (!AlignConstants.ALIGN_ENABLED) return;
        Pose2d pose = drivetrain.getState().Pose;
        OptionalInt id = AlignMath.nearestTag(pose, AlignConstants.PEG_TAG_IDS, tagPoses);
        if (id.isEmpty()) return;
        target = AlignMath.targetPose(tagPoses.get(id.getAsInt()),
            AlignConstants.OFFSET_DISTANCE_M, AlignConstants.OFFSET_LATERAL_M);
        var speeds = drivetrain.getState().Speeds;
        // Speeds are robot-relative; rotate into the field frame for the controller reset.
        var fieldSpeeds = edu.wpi.first.math.kinematics.ChassisSpeeds
            .fromRobotRelativeSpeeds(speeds, pose.getRotation());
        xPID.reset(pose.getX(), fieldSpeeds.vxMetersPerSecond);
        yPID.reset(pose.getY(), fieldSpeeds.vyMetersPerSecond);
        thetaPID.reset(pose.getRotation().getRadians(), speeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber("Align/TagId", id.getAsInt());
    }

    @Override
    public void execute() {
        if (target == null) return;
        Pose2d pose = drivetrain.getState().Pose;
        double now = Timer.getFPGATimestamp();
        boolean at = AlignMath.atTarget(pose, target,
            AlignConstants.XY_TOLERANCE_M, AlignConstants.THETA_TOLERANCE_RAD);
        var status = machine.update(now, at, lastVisionTimestamp.getAsDouble());
        SmartDashboard.putString("Align/Status", status.name());
        SmartDashboard.putBoolean("Align/Aligned", machine.isAligned(now));
        if (status != AlignStateMachine.Status.RUNNING) return;

        double vx = xPID.calculate(pose.getX(), target.getX()) + xPID.getSetpoint().velocity;
        double vy = yPID.calculate(pose.getY(), target.getY()) + yPID.getSetpoint().velocity;
        double w = thetaPID.calculate(pose.getRotation().getRadians(), target.getRotation().getRadians())
            + thetaPID.getSetpoint().velocity;
        double cap = AlignConstants.MAX_SPEED_MPS;
        double wCap = AlignConstants.MAX_ANGULAR_RAD_PER_SEC;
        drivetrain.setControl(request
            .withVelocityX(Math.max(-cap, Math.min(cap, vx)))
            .withVelocityY(Math.max(-cap, Math.min(cap, vy)))
            .withRotationalRate(Math.max(-wCap, Math.min(wCap, w))));
    }

    @Override
    public boolean isFinished() {
        return target == null || machine.getStatus() != AlignStateMachine.Status.RUNNING;
    }

    @Override
    public void end(boolean interrupted) {
        machine.reset();
        SmartDashboard.putBoolean("Align/Aligned", false);
    }
}
