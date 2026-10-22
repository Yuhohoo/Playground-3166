package frc.robot.align;

/** Pure alignment counter / timeout / lost-target logic with injected timestamps. */
public class AlignStateMachine {
    public enum Status { RUNNING, TIMED_OUT, LOST_TARGET, POSE_RESET }

    private final double timeoutSec;
    private final double lostTargetSec;
    private final int requiredLoops;

    private double startTime;
    private int consecutive;
    private boolean aligned;
    private double lastFreshVisionTime = Double.NEGATIVE_INFINITY;
    private Status status = Status.RUNNING;

    public AlignStateMachine(double timeoutSec, double lostTargetSec, int requiredLoops) {
        this.timeoutSec = timeoutSec;
        this.lostTargetSec = lostTargetSec;
        this.requiredLoops = requiredLoops;
    }

    public void start(double now) {
        startTime = now;
        consecutive = 0;
        aligned = false;
        status = Status.RUNNING;
        lastFreshVisionTime = Double.NEGATIVE_INFINITY;
    }

    /** Call once per loop. {@code lastVisionTimestamp} = newest accepted frame time (-inf if none). */
    public Status update(double now, boolean atTarget, double lastVisionTimestamp) {
        lastFreshVisionTime = lastVisionTimestamp;
        if (status != Status.RUNNING) { aligned = false; return status; }
        if (now - startTime >= timeoutSec) {
            status = Status.TIMED_OUT;
        } else if (!isFresh(now)) {
            status = Status.LOST_TARGET;
        }
        if (status != Status.RUNNING) {
            consecutive = 0;
            aligned = false;
            return status;
        }
        consecutive = atTarget ? consecutive + 1 : 0;
        aligned = consecutive >= requiredLoops;
        return status;
    }

    private boolean isFresh(double now) {
        return now - lastFreshVisionTime <= lostTargetSec;
    }

    /** True only if running, in tolerance for N loops, and vision is still fresh at {@code now}. */
    public boolean isAligned(double now) {
        return aligned && status == Status.RUNNING && isFresh(now);
    }

    public void poseReset() {
        status = Status.POSE_RESET;
        aligned = false;
        consecutive = 0;
    }

    public Status getStatus() { return status; }

    /** Clears alignment (call on command end). */
    public void reset() {
        consecutive = 0;
        aligned = false;
        status = Status.RUNNING;
        lastFreshVisionTime = Double.NEGATIVE_INFINITY;
    }
}
