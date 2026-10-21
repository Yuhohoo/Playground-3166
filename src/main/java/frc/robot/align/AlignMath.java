package frc.robot.align;

import java.util.Map;
import java.util.OptionalInt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

/** Pure target-pose math and tag selection. */
public final class AlignMath {
    private AlignMath() {}

    /**
     * Robot target: {@code distance} m in front of the tag face (along tag normal), {@code lateral}
     * m to the left as seen facing the tag, rotated to face the tag.
     */
    public static Pose2d targetPose(Pose2d tagPose, double distance, double lateral) {
        // Tag frame: +x out of the tag face. Viewer facing the tag has the tag's +y on their right,
        // so "left" from the viewer is the tag's -y.
        return tagPose.transformBy(new Transform2d(distance, -lateral, Rotation2d.k180deg));
    }

    /** Nearest allowed tag (by distance to the tag) that exists in {@code tagPoses}. */
    public static OptionalInt nearestTag(Pose2d robot, int[] allowedIds, Map<Integer, Pose2d> tagPoses) {
        int best = -1;
        double bestDist = Double.POSITIVE_INFINITY;
        for (int id : allowedIds) {
            Pose2d p = tagPoses.get(id);
            if (p == null) continue;
            double d = robot.getTranslation().getDistance(p.getTranslation());
            if (d < bestDist) { bestDist = d; best = id; }
        }
        return best < 0 ? OptionalInt.empty() : OptionalInt.of(best);
    }

    public static boolean atTarget(Pose2d robot, Pose2d target, double xyTol, double thetaTolRad) {
        double dist = robot.getTranslation().getDistance(target.getTranslation());
        double dTheta = Math.abs(target.getRotation().minus(robot.getRotation()).getRadians());
        return dist <= xyTol && dTheta <= thetaTolRad;
    }
}
