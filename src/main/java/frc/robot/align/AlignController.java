package frc.robot.align;

import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.IntToDoubleFunction;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.AlignConstants;

/** Hardware-free controller shared by the command and closed-loop tests. Outputs are blue-field SI units. */
public final class AlignController {
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


    private final AlignStateMachine machine = new AlignStateMachine(
        AlignConstants.TIMEOUT_SEC, AlignConstants.LOST_TARGET_SEC, AlignConstants.ALIGNED_LOOPS);
    private Pose2d previousPose;
    private double previousTime;
    private ChassisSpeeds previousCommand = new ChassisSpeeds();
    private Pose2d target;
    private int targetId = -1;
    private boolean active;
    private final Supplier<Optional<Alliance>> alliance;
    private final java.util.function.BooleanSupplier poseTrusted;
    public AlignController() { this(DriverStation::getAlliance, () -> false); }
    public AlignController(Supplier<Optional<Alliance>> alliance) {
        this(alliance, () -> false);
    }
    public AlignController(Supplier<Optional<Alliance>> alliance, java.util.function.BooleanSupplier poseTrusted) {
        this.poseTrusted = poseTrusted;
        this.alliance = alliance;
        thetaPID.enableContinuousInput(-Math.PI, Math.PI);
        xPID.setTolerance(AlignConstants.XY_TOLERANCE_M);
        yPID.setTolerance(AlignConstants.XY_TOLERANCE_M);
        thetaPID.setTolerance(AlignConstants.THETA_TOLERANCE_RAD);
    }
    public void start(double now, Pose2d pose, ChassisSpeeds fieldSpeeds,
            Map<Integer, Pose2d> tags, int[] pegIds) {
        stop();
        if (!poseTrusted.getAsBoolean()) { machine.poseUntrusted(); return; }
        // Alliance is a start gate only; all poses and outputs remain blue-origin.
        if (alliance.get().isEmpty()) return;
        machine.start(now);
        var id = AlignMath.nearestTag(pose, pegIds, tags);
        if (id.isEmpty()) return;
        previousPose = pose;
        previousTime = now;
        // Include initial measured motion before the first command has been sent.
        previousCommand = fieldSpeeds;
        active = true;
        targetId = id.getAsInt();
        target = AlignMath.targetPose(tags.get(targetId), AlignConstants.OFFSET_DISTANCE_M,
            AlignConstants.OFFSET_LATERAL_M);
        xPID.reset(pose.getX(), fieldSpeeds.vxMetersPerSecond);
        yPID.reset(pose.getY(), fieldSpeeds.vyMetersPerSecond);
        thetaPID.reset(pose.getRotation().getRadians(), fieldSpeeds.omegaRadiansPerSecond);
    }
    public ChassisSpeeds update(double now, Pose2d pose, IntToDoubleFunction freshness) {
        if (!active) return new ChassisSpeeds();
        if (machine.getStatus() != AlignStateMachine.Status.RUNNING) return new ChassisSpeeds();
        double dt = now - previousTime;
        double distance = pose.getTranslation().getDistance(previousPose.getTranslation());
        double angle = Math.abs(pose.getRotation().minus(previousPose.getRotation()).getRadians());
        double allowedDistance = Math.hypot(previousCommand.vxMetersPerSecond,
            previousCommand.vyMetersPerSecond) * dt + AlignConstants.POSE_DISCONTINUITY_M;
        double allowedAngle = Math.abs(previousCommand.omegaRadiansPerSecond) * dt
            + AlignConstants.POSE_DISCONTINUITY_RAD;
        if (!Double.isFinite(distance) || !Double.isFinite(angle) || !Double.isFinite(dt)
                || dt < 0 || distance > allowedDistance || angle > allowedAngle) {
            machine.poseReset();
            return new ChassisSpeeds();
        }
        previousPose = pose;
        previousTime = now;
        var status = machine.update(now, AlignMath.atTarget(pose, target,
            AlignConstants.XY_TOLERANCE_M, AlignConstants.THETA_TOLERANCE_RAD), freshness.applyAsDouble(targetId));
        if (status != AlignStateMachine.Status.RUNNING) return new ChassisSpeeds();
        double vx = xPID.calculate(pose.getX(), target.getX()) + xPID.getSetpoint().velocity;
        double vy = yPID.calculate(pose.getY(), target.getY()) + yPID.getSetpoint().velocity;
        double w = thetaPID.calculate(pose.getRotation().getRadians(), target.getRotation().getRadians())
            + thetaPID.getSetpoint().velocity;
        double cap = AlignConstants.MAX_SPEED_MPS;
        double wCap = AlignConstants.MAX_ANGULAR_RAD_PER_SEC;

        double norm = Math.hypot(vx, vy);
        if (norm > cap) { vx *= cap / norm; vy *= cap / norm; }
        previousCommand = new ChassisSpeeds(vx, vy, Math.max(-wCap, Math.min(wCap, w)));
        return new ChassisSpeeds(previousCommand.vxMetersPerSecond, previousCommand.vyMetersPerSecond,
            previousCommand.omegaRadiansPerSecond);
    }
    private Runnable poseResetHandler = () -> {};
    public void setPoseResetHandler(Runnable handler) {
        poseResetHandler = java.util.Objects.requireNonNull(handler);
    }
    public void stop() {
        if (machine.getStatus() == AlignStateMachine.Status.POSE_RESET) poseResetHandler.run();
        active = false; target = null; targetId = -1; machine.reset(); }
    public boolean isAligned(double now) { return active && machine.isAligned(now); }
    public boolean isFinished() { return !active || machine.getStatus() != AlignStateMachine.Status.RUNNING; }
    public int targetId() { return targetId; }
    public Pose2d target() { return target; }
    public AlignStateMachine.Status status() { return machine.getStatus(); }
}
