package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Testable gating of commands on alignment state. */
public final class AlignGating {
    private AlignGating() {}

    /**
     * Runs {@code shoot} only if {@code allowed} is true at start, and keeps checking it every loop:
     * the moment it drops, {@code shoot} is cancelled and {@code stop} runs (stop also runs on a
     * normal finish or interrupt).
     */
    public static Command gatedWhile(Command shoot, BooleanSupplier allowed, Runnable stop) {
        return Commands.either(
            shoot.onlyWhile(allowed).finallyDo(stop),
            Commands.none(),
            allowed);
    }

    /** Binds {@code seed} to {@code button} press, ignored while alignment is running. */
    public static void bindSeedUnlessAligning(Trigger button, Command seed, BooleanSupplier alignRunning) {
        button.and(() -> !alignRunning.getAsBoolean()).onTrue(seed);
    }
}
