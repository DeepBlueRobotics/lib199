package org.carlmontrobotics.lib199.safeMode;

import static org.junit.Assert.*;

import java.util.concurrent.atomic.AtomicInteger;

import org.junit.Test;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class SafeExecuteBlockingCommandTest {

    @Test
    public void testSafeExecuteBlockingCommand() {
        AtomicInteger counter = new AtomicInteger(0);

        SafeExecuteBlockingCommand command = new SafeExecuteBlockingCommand(new RunCommand(counter::incrementAndGet)) {
            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        };

        SafeMode.enable();
        command.schedule();
        CommandScheduler.getInstance().run();
        assertTrue(command.isScheduled());
        assertEquals(1, counter.get());

        SafeMode.disable();
        CommandScheduler.getInstance().run();
        assertTrue(command.isScheduled());
        assertEquals(1, counter.get());
    }

}
