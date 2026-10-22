package frc.robot.vision;

import java.util.HashMap;
import java.util.Map;

/** Per-camera lifetime totals, independent of pose trust and its consecutive-frame counters. */
public final class JumpRejectCounter {
    private final Map<String, Long> totals = new HashMap<>();
    private final Map<String, Double> lastExposure = new HashMap<>();

    public long record(String camera, double timestamp, VisionFilter.Result result) {
        if (result == VisionFilter.Result.DUPLICATE || !Double.isFinite(timestamp)) return get(camera);
        Double previous = lastExposure.put(camera, timestamp);
        if (previous != null && previous.doubleValue() == timestamp) return get(camera);
        if (result == VisionFilter.Result.POSE_JUMP) totals.merge(camera, 1L, Long::sum);
        return get(camera);
    }

    public long get(String camera) { return totals.getOrDefault(camera, 0L); }
}
