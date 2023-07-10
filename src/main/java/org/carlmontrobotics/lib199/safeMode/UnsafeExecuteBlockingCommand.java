package org.carlmontrobotics.lib199.safeMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A command that only runs its {@link #execute()} method when safe-mode is disabled, and continues running as long as the underlying command is not finished.
 *
 * Keep in mind that this does not block calls to {@link #initialize()} or {@link #end(boolean)}, so it is not appropriate for wrapping commands such as {@link edu.wpi.first.wpilibj2.command.InstantCommand}.
 * It is intended for cases where the command should keep running such as a default command where {@link #isFinished()} must always return {@code false}
 */
public class UnsafeExecuteBlockingCommand extends FunctionalCommand {

    private final Command command;

    static {
        SafeMode.ensureRegistered();
    }

    public UnsafeExecuteBlockingCommand(Command command) {
        super(
            command::initialize,
            () -> { if (!SafeMode.isEnabled()) command.execute(); },
            command::end,
            command::isFinished,
            command.getRequirements().toArray(Subsystem[]::new)
        );
        CommandScheduler.getInstance().registerComposedCommands(this.command = command);
    }

    @Override
    public boolean runsWhenDisabled() {
        return command.runsWhenDisabled();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return command.getInterruptionBehavior();
    }

}
