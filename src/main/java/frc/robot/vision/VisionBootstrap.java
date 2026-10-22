package frc.robot.vision;

/** Pose trust lifetime; defers the enable edge until triggers have been polled. */
public final class VisionBootstrap {
    private final java.util.function.BooleanSupplier enabled;
    private static final class SingleTagWindow {
        final java.util.ArrayDeque<edu.wpi.first.math.geometry.Translation2d> poses = new java.util.ArrayDeque<>();
        int id = -1;
    }
    private final java.util.Map<String, SingleTagWindow> windows = new java.util.HashMap<>();
    private final java.util.Map<String, Integer> jumpRejections = new java.util.HashMap<>();
    private boolean wasEnabled;
    private double disabledSince = Double.NaN;
    private final java.util.function.DoubleSupplier timestamp;

    public VisionBootstrap() { this(() -> false); }
    public VisionBootstrap(java.util.function.BooleanSupplier enabled) {
        this(enabled, edu.wpi.first.wpilibj.Timer::getFPGATimestamp);
    }
    public VisionBootstrap(java.util.function.BooleanSupplier enabled, java.util.function.DoubleSupplier timestamp) {
        this.enabled = enabled;
        this.timestamp = timestamp;
    }
    public boolean beginLoop() { return beginLoop(enabled.getAsBoolean()); }
    public boolean isPoseTrusted() { return bootstrapped; }
    public boolean isCollectingSingleTag(String camera) { return windows.containsKey(camera); }
    /** Missing or rejected exposures only break this camera's consecutive-frame streak. */
    public void rejectFrame(String camera) { windows.remove(camera); }
    /** Skip repeat exposures without changing stability; reject only fresh invalid frames. */
    public boolean acceptFrame(String camera, VisionFilter.Result result) {
        if (result == VisionFilter.Result.ACCEPT) return true;
        if (result != VisionFilter.Result.DUPLICATE) rejectFrame(camera);
        return false;
    }
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
        return beginLoop(enabled, timestamp.getAsDouble());
    }

    public boolean beginLoop(boolean enabled, double now) {
        boolean justEnabled = enabled && !wasEnabled;
        if (!enabled && (wasEnabled || Double.isNaN(disabledSince))) disabledSince = now;
        if (!Double.isNaN(disabledSince)
                && now - disabledSince > frc.robot.Constants.VisionConstants.POSE_TRUST_MAX_DISABLE_S)
            clearPoseTrust();
        if (enabled) disabledSince = Double.NaN;
        if (!enabled || justEnabled) windows.clear();
        wasEnabled = enabled;
        return enabled && !justEnabled;
    }

    public boolean shouldReset(boolean alignActive, int[] ids, int[] layoutIds,
            double distance, double ambiguity) {
        return VisionFilter.shouldBootstrap(bootstrapped, alignActive, ids, layoutIds, distance, ambiguity);
    }

    /** Called once per fresh exposure; rejected poses are never fused. */
    public VisionFilter.Result evaluatePoseJump(String camera, edu.wpi.first.math.geometry.Pose2d measured,
            edu.wpi.first.math.geometry.Pose2d estimated, boolean bootstrap) {
        var result = VisionFilter.evaluatePoseJump(measured, estimated, bootstrap);
        if (result == VisionFilter.Result.POSE_JUMP
                && jumpRejections.merge(camera, 1, Integer::sum)
                    >= frc.robot.Constants.VisionConstants.VISION_TRUST_DROP_FRAMES) clearPoseTrust();
        return result;
    }

    /** Only a fresh frame actually fused by this camera resets its rejection count. */
    public void didFuse(String camera) { jumpRejections.remove(camera); }

    private long trustDrops;
    public long getTrustDrops() { return trustDrops; }

    public void clearPoseTrust() {
        if (bootstrapped) trustDrops++;
        bootstrapped = false; windows.clear(); jumpRejections.clear();
    }

    public void didReset() { bootstrapped = true; windows.clear(); jumpRejections.clear(); }
}
