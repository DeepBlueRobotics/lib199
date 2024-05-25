package org.carlmontrobotics.lib199.safeMode;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class EndBlockingCommandTest {

    @Test
    public void testEndBlockingCommand() {
        LoggingCommand loggingCommand = new LoggingCommand();
        EndBlockingCommand command = new EndBlockingCommand(loggingCommand);
        loggingCommand.reset();
        command.initialize();
        command.end(false);
        assertEquals(1, loggingCommand.getEndCount());
        command.end(false);
        assertEquals(1, loggingCommand.getEndCount());
    }

}
