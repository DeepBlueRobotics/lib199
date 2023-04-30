package org.carlmontrobotics.lib199.safeMode;

import static org.junit.Assert.*;

import org.junit.Test;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class UnsafeCommandTest {

    @Test
    public void testUnsafeCommand() {
        UnsafeCommand command = new UnsafeCommand(new RunCommand(() -> {})) {
            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        };

        SafeMode.disable();
        command.schedule();
        CommandScheduler.getInstance().run();
        assertTrue(command.isScheduled());

        SafeMode.enable();
        CommandScheduler.getInstance().run();
        assertFalse(command.isScheduled());
    }

}
