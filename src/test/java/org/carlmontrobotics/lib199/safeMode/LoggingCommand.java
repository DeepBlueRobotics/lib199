package org.carlmontrobotics.lib199.safeMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

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

    public void reset() {
        initializedCount = executeCount = 0;
    }

    public int getInitializedCount() {
        return initializedCount;
    }

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
