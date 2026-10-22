package frc.robot;

import static org.junit.jupiter.api.Assertions.*;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.align.*;

class HandoffSchedulerTest {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private int loop;
    private boolean held, done;
    private double stick = .4;
    private final List<Integer> manualLoops = new ArrayList<>();
    private final List<Integer> holdLoops = new ArrayList<>();
    private SubsystemBase drive;
    private Command handoff;
    private AlignEndCommand align;
    private int starts;

    private void clean() {
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        scheduler.cancelAll();
        scheduler.unregisterAllSubsystems();
        scheduler.getActiveButtonLoop().clear();
        scheduler.getDefaultButtonLoop().clear();
        scheduler.setActiveButtonLoop(scheduler.getDefaultButtonLoop());
        DriverStationSim.resetData();
    }

    @BeforeEach void setup() {
        HAL.initialize(500, 0);
        clean();
        scheduler.enable();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        drive = new SubsystemBase() {};
        drive.setDefaultCommand(drive.run(() -> manualLoops.add(loop)));
        handoff = DriverHandoff.holdZero(() -> holdLoops.add(loop),
            () -> DriverHandoff.sticksNeutral(stick, 0, 0), drive);
        align = new AlignEndCommand() {
            { addRequirements(drive); }
            @Override public void initialize() { starts++; }
            @Override public boolean isFinished() { return done; }
            @Override public void end(boolean interrupted) {
                // Production stops its request before this same callback path.
                super.end(interrupted);
            }
        };
        DriverHandoff.wireEnd(align, handoff);
        new Trigger(() -> held).whileTrue(align);
        tick(); // Schedule the default, then let it execute before alignment starts.
        tick();
        assertFalse(manualLoops.isEmpty());
        held = true;
        tick();
        assertSame(align, scheduler.requiring(drive));
        manualLoops.clear();
        holdLoops.clear();
    }

    @AfterEach void cleanup() { clean(); }

    private void tick() { loop++; scheduler.run(); }

    private void verifyHoldAndResume() {
        tick(); // First scheduler loop after align ended.
        assertSame(handoff, scheduler.requiring(drive));
        assertTrue(holdLoops.contains(loop));
        for (int i = 0; i < 5; i++) tick();
        assertTrue(manualLoops.isEmpty(), "Default executed between align and neutral sticks");
        assertEquals(1, starts);
        held = false;
        tick();
        assertSame(handoff, scheduler.requiring(drive));
        stick = 0;
        tick();
        tick();
        assertFalse(handoff.isScheduled());
        assertFalse(manualLoops.isEmpty());
    }

    @Test void releaseHandsOffWithoutDefaultGap() {
        held = false;
        tick();
        assertFalse(align.isScheduled());
        verifyHoldAndResume();
    }

    @Test void selfCompletionWhileHeldHandsOffWithoutDefaultGap() {
        done = true;
        tick();
        assertFalse(align.isScheduled());
        verifyHoldAndResume();
    }

    @Test void disableEndsAlignWithoutHandoffOrDrive() {
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        for (int i = 0; i < 6; i++) tick();
        assertFalse(align.isScheduled());
        assertFalse(handoff.isScheduled());
        assertTrue(holdLoops.isEmpty());
        assertTrue(manualLoops.isEmpty());
    }
}
