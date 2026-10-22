package frc.robot.align;

import edu.wpi.first.wpilibj2.command.Command;

/** Align end callback path shared with scheduler tests without constructing CAN hardware. */
public abstract class AlignEndCommand extends Command {
    private Runnable endHandler = () -> {};

    public final void setEndHandler(Runnable handler) {
        endHandler = java.util.Objects.requireNonNull(handler);
    }

    /** Subclasses stop their output before invoking this callback. */
    @Override
    public void end(boolean interrupted) {
        endHandler.run();
    }
}
