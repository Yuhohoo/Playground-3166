package frc.robot.align;

/** Pure release handoff policy, matching the existing 10% manual request deadbands. */
public final class DriverHandoff {
    private DriverHandoff() {}

    public static boolean sticksNeutral(double leftX, double leftY, double rightX) {
        // FieldCentric translation deadband applies to the combined translation magnitude.
        return Double.isFinite(leftX) && Double.isFinite(leftY) && Double.isFinite(rightX)
            && Math.hypot(leftX, leftY) < 0.1 && Math.abs(rightX) < 0.1;
    }
}
