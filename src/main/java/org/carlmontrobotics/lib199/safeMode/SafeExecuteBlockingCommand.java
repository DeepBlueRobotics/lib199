package org.carlmontrobotics.lib199.safeMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

/**
 * A command that only runs its {@link #execute()} method when safe-mode is enabled, and continues running as long as the underlying command is not finished.
 *
 * Keep in mind that this does not block calls to {@link #initialize()} or {@link #end(boolean)}, so it is not appropriate for wrapping commands such as {@link edu.wpi.first.wpilibj2.command.InstantCommand}.
 * It is intended for cases where the command should keep running such as a default command where {@link #isFinished()} must always return {@code false}
 */
public class SafeExecuteBlockingCommand extends WrapperCommand {

    public SafeExecuteBlockingCommand(Command command) {
        super(command);
    }

    @Override
    public void execute() {
        if(SafeMode.isEnabled()) super.execute();
    }

}
