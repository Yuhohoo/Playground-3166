package frc.robot.align;

import java.util.Map;
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
    private Pose2d target;
    private int targetId = -1;
    private boolean active;
    public AlignController() {
        thetaPID.enableContinuousInput(-Math.PI, Math.PI);
        xPID.setTolerance(AlignConstants.XY_TOLERANCE_M);
        yPID.setTolerance(AlignConstants.XY_TOLERANCE_M);
        thetaPID.setTolerance(AlignConstants.THETA_TOLERANCE_RAD);
    }
    public void start(double now, Pose2d pose, ChassisSpeeds fieldSpeeds,
            Map<Integer, Pose2d> tags, int[] pegIds) {
        stop();
        machine.start(now);
        var id = AlignMath.nearestTag(pose, pegIds, tags);
        if (id.isEmpty()) return;
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
        var status = machine.update(now, AlignMath.atTarget(pose, target,
            AlignConstants.XY_TOLERANCE_M, AlignConstants.THETA_TOLERANCE_RAD), freshness.applyAsDouble(targetId));
        if (status != AlignStateMachine.Status.RUNNING) return new ChassisSpeeds();
        double vx = xPID.calculate(pose.getX(), target.getX()) + xPID.getSetpoint().velocity;
        double vy = yPID.calculate(pose.getY(), target.getY()) + yPID.getSetpoint().velocity;
        double w = thetaPID.calculate(pose.getRotation().getRadians(), target.getRotation().getRadians())
            + thetaPID.getSetpoint().velocity;
        double cap = AlignConstants.MAX_SPEED_MPS;
        double wCap = AlignConstants.MAX_ANGULAR_RAD_PER_SEC;

        return new ChassisSpeeds(Math.max(-cap, Math.min(cap, vx)),
            Math.max(-cap, Math.min(cap, vy)), Math.max(-wCap, Math.min(wCap, w)));
    }
    public void stop() { active = false; target = null; targetId = -1; machine.reset(); }
    public boolean isAligned(double now) { return active && machine.isAligned(now); }
    public boolean isFinished() { return !active || machine.getStatus() != AlignStateMachine.Status.RUNNING; }
    public int targetId() { return targetId; }
    public Pose2d target() { return target; }
    public AlignStateMachine.Status status() { return machine.getStatus(); }
}
