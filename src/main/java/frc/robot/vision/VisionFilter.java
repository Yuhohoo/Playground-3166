package frc.robot.vision;

import frc.robot.Constants.VisionConstants;

/** Pure (HAL-free) accept/reject and std-dev logic for vision frames. */
public final class VisionFilter {
    private VisionFilter() {}

    public enum Result { ACCEPT, POSE_JUMP, UNKNOWN_ID, NO_TAGS, SPINNING, STALE, DUPLICATE, INVALID }

    public static Result evaluate(int tagCount, double yawRateDegPerSec, double frameTimestampSec,
            double nowSec, double lastAcceptedTimestampSec, double maxYawRateDegPerSec, double maxAgeSec) {
        if (Double.isNaN(frameTimestampSec) || Double.isNaN(yawRateDegPerSec)) return Result.INVALID;
        if (tagCount <= 0) return Result.NO_TAGS;
        if (Math.abs(yawRateDegPerSec) > maxYawRateDegPerSec) return Result.SPINNING;
        if (nowSec - frameTimestampSec > maxAgeSec) return Result.STALE;
        if (frameTimestampSec == lastAcceptedTimestampSec) return Result.DUPLICATE;
        return Result.ACCEPT;
    }

    public static Result evaluate(int tagCount, double yawRateDegPerSec, double frameTimestampSec,
            double nowSec, double lastAcceptedTimestampSec) {
        return evaluate(tagCount, yawRateDegPerSec, frameTimestampSec, nowSec, lastAcceptedTimestampSec,
            VisionConstants.MAX_YAW_RATE_DEG_PER_SEC, VisionConstants.MAX_FRAME_AGE_SEC);
    }

    /** Compare raw blue-origin measurement translation against the current estimate. */
    public static Result evaluatePoseJump(edu.wpi.first.math.geometry.Pose2d measurement,
            edu.wpi.first.math.geometry.Pose2d estimated) {
        double distance = measurement.getTranslation().getDistance(estimated.getTranslation());
        if (!Double.isFinite(distance)) return Result.INVALID;
        return distance > VisionConstants.VISION_MAX_POSE_JUMP_M ? Result.POSE_JUMP : Result.ACCEPT;
    }

    /** During alignment, eligible frames use the normal jump gate and never reset pose. */
    public static boolean shouldBootstrap(boolean bootstrapped, boolean alignActive, int[] ids, int[] layoutIds) {
        return !alignActive && shouldBootstrap(bootstrapped, ids, layoutIds);
    }

    public static boolean shouldBootstrap(boolean bootstrapped, int[] ids, int[] layoutIds) {
        return !bootstrapped && evaluateIds(ids, layoutIds) == Result.ACCEPT
            && java.util.Arrays.stream(ids).distinct().count() >= 2;
    }

    /** Scheduled alignment always uses the normal jump gate, even on the first enable frame. */
    public static boolean shouldBootstrap(boolean bootstrapped, boolean alignActive, int[] ids,
            int[] layoutIds, double singleDistance, double singleAmbiguity) {
        if (bootstrapped || alignActive || evaluateIds(ids, layoutIds) != Result.ACCEPT) return false;
        if (java.util.Arrays.stream(ids).distinct().count() >= 2) return true;
        return ids.length == 1 && Double.isFinite(singleDistance) && singleDistance >= 0
            && singleDistance < VisionConstants.VISION_SINGLE_TAG_RESET_MAX_DIST_M
            && Double.isFinite(singleAmbiguity) && singleAmbiguity >= 0
            && singleAmbiguity < VisionConstants.VISION_SINGLE_TAG_MAX_AMBIGUITY;
    }

    public static Result evaluatePoseJump(edu.wpi.first.math.geometry.Pose2d measured,
            edu.wpi.first.math.geometry.Pose2d estimated, boolean bootstrap) {
        var result = evaluatePoseJump(measured, estimated);
        return bootstrap && result == Result.POSE_JUMP ? Result.ACCEPT : result;
    }

    /** All fiducials must belong to the loaded layout; peg selection is independent. */
    public static Result evaluateIds(int[] ids, int[] layoutIds) {
        if (ids.length == 0) return Result.NO_TAGS;
        for (int id : ids) if (!contains(layoutIds, id)) return Result.UNKNOWN_ID;
        return Result.ACCEPT;
    }

    /** Number of {@code ids} that are in {@code allowed}; disallowed IDs never count as tags. */
    public static int countAllowed(int[] ids, int[] allowed) {
        int n = 0;
        for (int id : ids) if (contains(allowed, id)) n++;
        return n;
    }

    /** Only the allowed IDs, in order. */
    public static int[] filterAllowed(int[] ids, int[] allowed) {
        return java.util.Arrays.stream(ids).filter(id -> contains(allowed, id)).toArray();
    }

    public static boolean contains(int[] ids, int id) {
        for (int x : ids) if (x == id) return true;
        return false;
    }

    /** xy std dev (m) = base * (1 + d^2 / k). */
    public static double xyStdDev(double avgTagDistM, double base, double k) {
        return base * (1.0 + avgTagDistM * avgTagDistM / k);
    }

    /** @return {x, y, theta} std devs. */
    public static double[] stdDevs(double avgTagDistM) {
        double xy = xyStdDev(avgTagDistM, VisionConstants.BASE_XY_STD_DEV_M, VisionConstants.DISTANCE_SCALE_K);
        return new double[] {xy, xy, VisionConstants.THETA_STD_DEV_RAD};
    }
}
