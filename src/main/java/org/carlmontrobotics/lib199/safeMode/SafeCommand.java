package org.carlmontrobotics.lib199.safeMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SafeCommand extends FunctionalCommand {

    public SafeCommand(Command command) {
        super(
            command::initialize,
            command::execute,
            command::end,
            () -> command.isFinished() || !SafeMode.isEnabled(),
            command.getRequirements().toArray(Subsystem[]::new)
        );
        CommandScheduler.getInstance().registerComposedCommands(command);
    }

}
