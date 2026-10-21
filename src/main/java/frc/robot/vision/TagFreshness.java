package frc.robot.vision;

import java.util.HashMap;
import java.util.Map;

/** Per-tag-ID timestamp of the newest accepted frame that contained that tag. */
public class TagFreshness {
    private final Map<Integer, Double> last = new HashMap<>();

    public void record(int[] tagIds, double timestampSec) {
        for (int id : tagIds) last.merge(id, timestampSec, Math::max);
    }

    /** -inf if the tag has never been seen. */
    public double get(int tagId) {
        return last.getOrDefault(tagId, Double.NEGATIVE_INFINITY);
    }
}
