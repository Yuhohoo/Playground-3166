package frc.robot.vision;

/** Pure sampled heading-offset detector; all angles are injected in degrees.
 * Assumes effectively infinite vision theta std dev, so fusion cannot materially move heading.
 * Resets smaller than the threshold, or offset changes between samples that cancel, can evade it.
 */
public final class HeadingResetDetector {
    private final double thresholdDeg;
    private boolean initialized;
    private double lastPoseDeg;
    private double lastRawDeg;

    public HeadingResetDetector(double thresholdDeg) { this.thresholdDeg = thresholdDeg; }

    /** Bootstrap loops skip detection and establish a new post-reset baseline. */
    public boolean update(double poseDeg, double rawDeg, boolean bootstrapReset) {
        double deltaPose = wrap(poseDeg - lastPoseDeg);
        double deltaGyro = wrap(rawDeg - lastRawDeg);
        boolean reset = initialized && !bootstrapReset
            && Math.abs(deltaPose - deltaGyro) > thresholdDeg;
        lastPoseDeg = poseDeg;
        lastRawDeg = rawDeg;
        initialized = true;
        return reset;
    }

    private static double wrap(double degrees) {
        return Math.toDegrees(Math.atan2(Math.sin(Math.toRadians(degrees)),
            Math.cos(Math.toRadians(degrees))));
    }
}
