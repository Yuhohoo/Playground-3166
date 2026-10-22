package frc.robot.vision;

/** Per-enable bootstrap lifetime; defers the enable edge until triggers have been polled. */
public final class VisionBootstrap {
    private boolean wasEnabled;
    private boolean bootstrapped;

    public boolean beginLoop(boolean enabled) {
        boolean justEnabled = enabled && !wasEnabled;
        if (justEnabled) bootstrapped = false;
        wasEnabled = enabled;
        return enabled && !justEnabled;
    }

    public boolean shouldReset(boolean alignActive, int[] ids, int[] layoutIds,
            double distance, double ambiguity) {
        return VisionFilter.shouldBootstrap(bootstrapped, alignActive, ids, layoutIds, distance, ambiguity);
    }

    public void didReset() { bootstrapped = true; }
}
