package org.carlmontrobotics.lib199.safeMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

/**
 * A command who's {@link #end(boolean)} method can only be called if its {@link #initialize()} method has been called first.
 */
public class EndBlockingCommand extends WrapperCommand {

    private boolean initialized = false;

    /**
     * Creates a new EndBlockingCommand
     * @param command The command to run
     */
    public EndBlockingCommand(Command command) {
        super(command);
    }

    @Override
    public void initialize() {
        synchronized(this) {
            initialized = true;
        }
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        synchronized(this) {
            if(initialized) {
                super.end(interrupted);
                initialized = false;
            }
        }
    }

}
