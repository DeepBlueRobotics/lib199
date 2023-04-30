package org.carlmontrobotics.lib199.safeMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

// These are suitable for cases where the command should keep running such as a default command where isFinished must always return false
public class SafeExecuteBlockingCommand extends FunctionalCommand {

    public SafeExecuteBlockingCommand(Command command) {
        super(
            command::initialize,
            () -> { if (SafeMode.isEnabled()) command.execute(); },
            command::end,
            command::isFinished,
            command.getRequirements().toArray(Subsystem[]::new)
        );
        CommandScheduler.getInstance().registerComposedCommands(command);
    }

}