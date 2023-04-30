package org.carlmontrobotics.lib199.safeMode;

import static org.junit.Assert.*;

import java.util.concurrent.atomic.AtomicInteger;

import org.junit.Test;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class UnsafeExecuteBlockingCommandTest {

    @Test
    public void testUnsafeExecuteBlockingCommand() {
        AtomicInteger counter = new AtomicInteger(0);

        UnsafeExecuteBlockingCommand command = new UnsafeExecuteBlockingCommand(new RunCommand(counter::incrementAndGet)) {
            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        };

        SafeMode.disable();
        command.schedule();
        CommandScheduler.getInstance().run();
        assertTrue(command.isScheduled());
        assertEquals(1, counter.get());

        SafeMode.enable();
        CommandScheduler.getInstance().run();
        assertTrue(command.isScheduled());
        assertEquals(1, counter.get());
    }

}
