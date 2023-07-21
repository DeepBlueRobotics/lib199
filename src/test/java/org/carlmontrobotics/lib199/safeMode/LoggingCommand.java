package org.carlmontrobotics.lib199.safeMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A command that counts how many times it has been initialized and executed for the purposes of testing safe mode functionality.
 *
 * @see SafeModeCommandsTest
 */
public class LoggingCommand extends CommandBase {

    private int initializedCount = 0, executeCount = 0;

    @Override
    public void initialize() {
        initializedCount++;
    }

    @Override
    public void execute() {
        executeCount++;
    }

    /**
     * Resets the initialized and execute counts to zero.
     */
    public void reset() {
        initializedCount = executeCount = 0;
    }

    /**
     * @return The number of times this command has been initialized.
     */
    public int getInitializedCount() {
        return initializedCount;
    }

    /**
     * @return The number of times this command has been executed.
     */
    public int getExecuteCount() {
        return executeCount;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}