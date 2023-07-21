package org.carlmontrobotics.lib199.safeMode;

import static org.junit.Assert.*;

import java.util.function.Function;

import org.junit.Test;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class SafeModeCommandsTest {

    @Test
    public void testSafeCommand() {
        testCommand(SafeCommand::new, true, false);
    }

    @Test
    public void testSafeBlockingCommand() {
        testCommand(SafeExecuteBlockingCommand::new, true, true);
    }

    @Test
    public void testUnsafeCommand() {
        testCommand(UnsafeCommand::new, false, false);
    }

    @Test
    public void testUnsafeExecuteBlockingCommand() {
        testCommand(UnsafeExecuteBlockingCommand::new, false, true);
    }

    public void testCommand(Function<Command, Command> constructor, boolean isSafe, boolean staysEnabled) {
        LoggingCommand loggingCommand = new LoggingCommand();
        Command command = constructor.apply(loggingCommand);

        loggingCommand.reset();

        SafeMode.enable();
        command.schedule();
        CommandScheduler.getInstance().run();
        if (isSafe || staysEnabled)
            assertTrue(command.isScheduled());
        else
            assertFalse(command.isScheduled());
        if (!staysEnabled) {
            assertEquals(isSafe ? 1 : 0, loggingCommand.getInitializedCount());
            assertEquals(0, loggingCommand.getEndCount());
        }
        assertEquals(isSafe ? 1 : 0, loggingCommand.getExecuteCount());

        SafeMode.disable();
        command.schedule();
        CommandScheduler.getInstance().run();
        if (!isSafe || staysEnabled)
            assertTrue(command.isScheduled());
        else
            assertFalse(command.isScheduled());
        if (!staysEnabled) {
            assertEquals(1, loggingCommand.getInitializedCount());
            if(isSafe)
                assertEquals(1, loggingCommand.getEndCount());
            else
                assertEquals(0, loggingCommand.getEndCount());
        }
        assertEquals(1, loggingCommand.getExecuteCount());

        if(!staysEnabled && !isSafe) {
            SafeMode.enable();
            CommandScheduler.getInstance().run();
            assertEquals(1, loggingCommand.getEndCount());
        }
    }

}
