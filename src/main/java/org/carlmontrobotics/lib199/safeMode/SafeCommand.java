package org.carlmontrobotics.lib199.safeMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

/**
 * A command that only runs when safe-mode is enabled and returns {@code isFinished() = true} otherwise. If safe-mode is disabled, {@code end(true)} will be called.
 */
public class SafeCommand extends WrapperCommand {

    static {
        SafeMode.ensureInitialized();
    }

    /**
     * Creates a new SafeCommand
     * @param command The command to run
     */
    public SafeCommand(Command command) {
        super(new EndBlockingCommand(command));
    }

    @Override
    public void initialize() {
        if(SafeMode.isEnabled()) super.initialize();
    }

    @Override
    public void execute() {
        if(SafeMode.isEnabled()) super.execute();
    }

    @Override
    public boolean isFinished() {
        if(!SafeMode.isEnabled()) end(true);
        return super.isFinished() || !SafeMode.isEnabled();
    }
}
