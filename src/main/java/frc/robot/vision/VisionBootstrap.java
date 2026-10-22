package frc.robot.vision;

/** Per-enable bootstrap lifetime; defers the enable edge until triggers have been polled. */
public final class VisionBootstrap {
    private final java.util.function.BooleanSupplier enabled;
    private static final class SingleTagWindow {
        final java.util.ArrayDeque<edu.wpi.first.math.geometry.Translation2d> poses = new java.util.ArrayDeque<>();
        int id = -1;
    }
    private final java.util.Map<String, SingleTagWindow> windows = new java.util.HashMap<>();
    private boolean wasEnabled;

    public VisionBootstrap() { this(() -> false); }
    public VisionBootstrap(java.util.function.BooleanSupplier enabled) { this.enabled = enabled; }
    public boolean beginLoop() { return beginLoop(enabled.getAsBoolean()); }
    public boolean isPoseTrusted() { return bootstrapped; }
    public boolean isCollectingSingleTag(String camera) { return windows.containsKey(camera); }
    /** Missing or rejected exposures only break this camera's consecutive-frame streak. */
    public void rejectFrame(String camera) { windows.remove(camera); }
    public void observeAlign(boolean active) { if (active) windows.clear(); }

    /** Called only after frame validation, before the bootstrap-specific jump gate. */
    public boolean shouldReset(String camera, boolean alignActive, int[] ids, int[] layoutIds,
            double distance, double ambiguity, edu.wpi.first.math.geometry.Pose2d pose) {
        observeAlign(alignActive);
        if (!shouldReset(alignActive, ids, layoutIds, distance, ambiguity)
                || !Double.isFinite(pose.getX()) || !Double.isFinite(pose.getY())) {
            rejectFrame(camera);
            return false;
        }
        if (ids.length != 1) { rejectFrame(camera); return true; }
        var candidate = windows.computeIfAbsent(camera, key -> new SingleTagWindow());
        if (candidate.id != ids[0]) candidate.poses.clear();
        candidate.id = ids[0];
        var window = candidate.poses;
        window.addLast(pose.getTranslation());
        while (window.size() > frc.robot.Constants.VisionConstants.VISION_SINGLE_TAG_STABLE_FRAMES)
            window.removeFirst();
        for (var a : window) for (var b : window)
            if (a.getDistance(b) > frc.robot.Constants.VisionConstants.VISION_SINGLE_TAG_MAX_SPREAD_M)
                return false;
        return window.size() == frc.robot.Constants.VisionConstants.VISION_SINGLE_TAG_STABLE_FRAMES;
    }

    private boolean bootstrapped;

    public boolean beginLoop(boolean enabled) {
        boolean justEnabled = enabled && !wasEnabled;
        if (justEnabled) bootstrapped = false;
        if (!enabled || justEnabled) windows.clear();
        wasEnabled = enabled;
        return enabled && !justEnabled;
    }

    public boolean shouldReset(boolean alignActive, int[] ids, int[] layoutIds,
            double distance, double ambiguity) {
        return VisionFilter.shouldBootstrap(bootstrapped, alignActive, ids, layoutIds, distance, ambiguity);
    }

    public void didReset() { bootstrapped = true; windows.clear(); }
}
