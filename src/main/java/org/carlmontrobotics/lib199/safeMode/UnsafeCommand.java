package org.carlmontrobotics.lib199.safeMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A command that only runs when safe-mode is disabled and returns {@code isFinished() = true} otherwise.
 *
 * Note that this does not block calls to {@link #end(boolean)} (If the command is scheduled when safe-mode is enabled, {@link #end(boolean)} will be called immediately})
 */
public class UnsafeCommand extends FunctionalCommand {

    private final Command command;

    static {
        SafeMode.ensureInitialized();
    }

    public UnsafeCommand(Command command) {
        super(
            () -> { if(!SafeMode.isEnabled()) command.initialize(); },
            () -> { if(!SafeMode.isEnabled()) command.execute(); },
            command::end,
            () -> command.isFinished() || SafeMode.isEnabled(),
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