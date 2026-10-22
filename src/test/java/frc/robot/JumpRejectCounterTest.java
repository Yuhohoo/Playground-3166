package frc.robot;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import frc.robot.vision.JumpRejectCounter;
import frc.robot.vision.VisionFilter.Result;

class JumpRejectCounterTest {
    @Test void totalsArePerCameraAndDuplicatesDoNotCountOrReset() {
        var counter = new JumpRejectCounter();
        assertEquals(0, counter.get("limelight-a"));
        assertEquals(1, counter.record("limelight-a", 1, Result.POSE_JUMP));
        assertEquals(1, counter.record("limelight-a", 1, Result.POSE_JUMP));
        assertEquals(1, counter.record("limelight-a", 1, Result.DUPLICATE));
        assertEquals(1, counter.record("limelight-b", 1, Result.POSE_JUMP));
        assertEquals(1, counter.record("limelight-a", 2, Result.ACCEPT));
        assertEquals(2, counter.record("limelight-a", 3, Result.POSE_JUMP));
        assertEquals(1, counter.get("limelight-b"));
    }
}
