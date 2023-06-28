package org.carlmontrobotics.lib199.safeMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class UnsafeCommand extends FunctionalCommand {

    private final Command command;

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
