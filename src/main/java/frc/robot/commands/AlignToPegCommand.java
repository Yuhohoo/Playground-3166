package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;
import java.util.function.IntToDoubleFunction;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AlignConstants;
import frc.robot.align.FieldLayoutLoader;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Hold-to-align to the nearest peg tag. Ends NOT aligned on timeout or lost vision. */
public class AlignToPegCommand extends frc.robot.align.AlignEndCommand {
    private final CommandSwerveDrivetrain drivetrain;
    /** Newest fresh frame time for a given tag ID (-inf if none). */
    private final IntToDoubleFunction lastVisionTimestamp;
    private final Map<Integer, Pose2d> tagPoses = new HashMap<>();

    // Blue-origin field-centric: no operator-perspective flip.
    private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
        .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance);

    private final frc.robot.align.AlignController controller;

    public AlignToPegCommand(CommandSwerveDrivetrain drivetrain, IntToDoubleFunction lastVisionTimestamp,
            java.util.function.BooleanSupplier poseTrusted) {
        this(drivetrain, lastVisionTimestamp, poseTrusted, edu.wpi.first.wpilibj.DriverStation::getAlliance);
    }

    public AlignToPegCommand(CommandSwerveDrivetrain drivetrain, IntToDoubleFunction lastVisionTimestamp,
            java.util.function.BooleanSupplier poseTrusted,
            java.util.function.Supplier<java.util.Optional<edu.wpi.first.wpilibj.DriverStation.Alliance>> alliance) {
        controller = new frc.robot.align.AlignController(alliance, poseTrusted);
        this.drivetrain = drivetrain;
        this.lastVisionTimestamp = lastVisionTimestamp;
        if (AlignConstants.ALIGN_ENABLED) {
            FieldLayoutLoader.getDeployLayout().ifPresent(layout -> {
                for (int id : AlignConstants.PEG_TAG_IDS) {
                    layout.getTagPose(id).ifPresent(p -> tagPoses.put(id, p.toPose2d()));
                }
            });
        }
        addRequirements(drivetrain);
    }

    /** Invoked on end with POSE_RESET before the controller status is cleared. */
    public void setPoseResetHandler(Runnable handler) { controller.setPoseResetHandler(handler); }

    public boolean isAligned() {
        return isScheduled() && controller.isAligned(Timer.getFPGATimestamp());
    }

    @Override
    public void initialize() {
        controller.stop();
        drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        if (!AlignConstants.ALIGN_ENABLED || !FieldLayoutLoader.isAvailable()) return;
        var state = drivetrain.getState();
        controller.start(Timer.getFPGATimestamp(), state.Pose,
            edu.wpi.first.math.kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation()),
            tagPoses, AlignConstants.PEG_TAG_IDS);
        SmartDashboard.putNumber("Align/TagId", controller.targetId());
    }

    @Override
    public void execute() {
        var output = controller.update(Timer.getFPGATimestamp(), drivetrain.getState().Pose, lastVisionTimestamp);
        SmartDashboard.putString("Align/Status", controller.status().name());
        SmartDashboard.putBoolean("Align/Aligned", isAligned());
        drivetrain.setControl(request.withVelocityX(output.vxMetersPerSecond)
            .withVelocityY(output.vyMetersPerSecond).withRotationalRate(output.omegaRadiansPerSecond));
    }

    @Override
    public boolean isFinished() { return controller.isFinished(); }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Align/EndReason", interrupted ? "INTERRUPTED" : controller.status().name());
        controller.stop();
        drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        super.end(interrupted);
        SmartDashboard.putBoolean("Align/Aligned", false);
    }
}
