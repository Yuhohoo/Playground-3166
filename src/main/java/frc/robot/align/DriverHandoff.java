package frc.robot.align;

/** Pure release handoff policy, matching the existing 10% manual request deadbands. */
public final class DriverHandoff {
    private DriverHandoff() {}

    public static edu.wpi.first.wpilibj2.command.Command holdZero(Runnable zero,
            java.util.function.BooleanSupplier neutral, edu.wpi.first.wpilibj2.command.Subsystem drive) {
        Runnable hold = () -> { zero.run(); publish(!neutral.getAsBoolean()); };
        return new edu.wpi.first.wpilibj2.command.FunctionalCommand(
            hold, hold, interrupted -> publish(false), neutral, drive);
    }

    /** An enabled align end arms the next poll, before default command execution. */
    public static void wireEnd(AlignEndCommand align, edu.wpi.first.wpilibj2.command.Command handoff) {
        boolean[] ended = {false};
        align.setEndHandler(() -> ended[0] = edu.wpi.first.wpilibj.DriverStation.isEnabled());
        new edu.wpi.first.wpilibj2.command.button.Trigger(() -> {
            if (!edu.wpi.first.wpilibj.DriverStation.isEnabled()) {
                ended[0] = false;
                return false;
            }
            return ended[0] && !align.isScheduled();
        }).onTrue(handoff);
    }

    private static void publish(boolean holding) {
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Align/HandoffHoldingZero", holding);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Align/HandoffReason",
            holding ? "Release sticks to drive" : "");
    }

    public static boolean sticksNeutral(double leftX, double leftY, double rightX) {
        // FieldCentric translation deadband applies to the combined translation magnitude.
        return Double.isFinite(leftX) && Double.isFinite(leftY) && Double.isFinite(rightX)
            && Math.hypot(leftX, leftY) < 0.1 && Math.abs(rightX) < 0.1;
    }
}
