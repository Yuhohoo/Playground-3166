package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignGating;
import frc.robot.vision.TagFreshness;
import frc.robot.vision.VisionFilter;

class GatingAndFreshnessTest {
    @BeforeEach
    void setUp() {
        assertTrue(HAL.initialize(500, 0));
        edu.wpi.first.wpilibj.simulation.DriverStationSim.setEnabled(true);
        edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData();
        CommandScheduler.getInstance().cancelAll();
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().getDefaultButtonLoop().clear();
    }

    @Test
    void droppingAlignmentMidShotEndsCommandAndStops() {
        var sched = CommandScheduler.getInstance();
        var aligned = new AtomicBoolean(true);
        var stops = new AtomicInteger();
        var executes = new AtomicInteger();
        Command shoot = Commands.run(executes::incrementAndGet);
        Command cmd = AlignGating.gatedWhile(shoot, aligned::get, stops::incrementAndGet);

        sched.schedule(cmd);
        sched.run();
        sched.run();
        assertTrue(cmd.isScheduled());
        assertEquals(0, stops.get());

        aligned.set(false);
        sched.run();
        assertFalse(cmd.isScheduled());
        assertEquals(1, stops.get());
        int n = executes.get();
        sched.run();
        assertEquals(n, executes.get());
    }

    @Test
    void notAllowedAtStartNeverRuns() {
        var sched = CommandScheduler.getInstance();
        var stops = new AtomicInteger();
        var executes = new AtomicInteger();
        Command cmd = AlignGating.gatedWhile(Commands.run(executes::incrementAndGet), () -> false,
            stops::incrementAndGet);
        sched.schedule(cmd);
        sched.run();
        assertEquals(0, executes.get());
        assertFalse(cmd.isScheduled());
    }

    @Test
    void seedBlockedWhileAlignRunning() {
        var sched = CommandScheduler.getInstance();
        var pressed = new AtomicBoolean(false);
        var aligning = new AtomicBoolean(true);
        var seeds = new AtomicInteger();
        AlignGating.bindSeedUnlessAligning(new Trigger(pressed::get),
            Commands.runOnce(seeds::incrementAndGet), aligning::get);
        sched.run();
        pressed.set(true);
        sched.run();
        assertEquals(0, seeds.get());
        pressed.set(false);
        sched.run();
        aligning.set(false);
        pressed.set(true);
        sched.run();
        assertEquals(1, seeds.get());
    }

    @Test
    void triggerHeldAtEnableCannotBootstrapBeforeScheduling() {
        var scheduler = CommandScheduler.getInstance();
        var held = new AtomicBoolean(false);
        Command align = Commands.run(() -> {});
        var policy = new frc.robot.vision.VisionBootstrap();
        var resets = new AtomicInteger();
        var vision = new edu.wpi.first.wpilibj2.command.SubsystemBase() {
            @Override public void periodic() {
                if (policy.beginLoop(edu.wpi.first.wpilibj.DriverStation.isEnabled())
                        && policy.shouldReset(align.isScheduled(), new int[]{4,8}, new int[]{4,8}, 1, .1)) {
                    resets.incrementAndGet();
                    policy.didReset();
                }
            }
        };
        try {
            edu.wpi.first.wpilibj.simulation.DriverStationSim.setEnabled(false);
            edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData();
            new Trigger(held::get).whileTrue(align);
            scheduler.run();
            held.set(true);
            edu.wpi.first.wpilibj.simulation.DriverStationSim.setEnabled(true);
            edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData();
            scheduler.run(); // periodic precedes trigger poll: enable edge must be deferred.
            assertTrue(align.isScheduled());
            for (int i=0; i<5; i++) scheduler.run();
            assertEquals(0, resets.get());
            held.set(false);
            scheduler.run();
            scheduler.run();
            assertEquals(1, resets.get()); // Pending bootstrap succeeds only after release.
        } finally {
            scheduler.unregisterSubsystem(vision);
        }
    }

    @Test
    void fusionUsesWholeLayoutAndFreshnessUsesPegs() {
        int[] layout = {1, 4, 8};
        assertEquals(VisionFilter.Result.ACCEPT, VisionFilter.evaluateIds(new int[]{1}, layout));
        assertEquals(VisionFilter.Result.ACCEPT, VisionFilter.evaluateIds(new int[]{1,4}, layout));
        assertEquals(VisionFilter.Result.UNKNOWN_ID, VisionFilter.evaluateIds(new int[]{4,99}, layout));
        assertEquals(VisionFilter.Result.UNKNOWN_ID, VisionFilter.evaluateIds(new int[]{99}, layout));
        assertEquals(VisionFilter.Result.NO_TAGS, VisionFilter.evaluateIds(new int[]{}, layout));
        assertArrayEquals(new int[]{4}, VisionFilter.filterAllowed(new int[]{1,4}, new int[]{4,8}));
        assertArrayEquals(new int[]{}, VisionFilter.filterAllowed(new int[]{1}, new int[]{4,8}));
        assertEquals(VisionFilter.Result.ACCEPT, VisionFilter.evaluateIds(new int[]{4}, layout));
        assertArrayEquals(new int[]{}, VisionFilter.filterAllowed(new int[]{4}, new int[]{}));
    }

    @Test
    void freshnessTracksTargetTagOnly() {
        var f = new TagFreshness();
        assertEquals(Double.NEGATIVE_INFINITY, f.get(3));
        f.record(new int[] {3, 4}, 1.0);
        f.record(new int[] {4}, 2.0); // frame without tag 3
        assertEquals(1.0, f.get(3));
        assertEquals(2.0, f.get(4));
        f.record(new int[] {3}, 0.5); // older frame doesn't regress
        assertEquals(1.0, f.get(3));
        assertEquals(Double.NEGATIVE_INFINITY, f.get(9));
    }
}
