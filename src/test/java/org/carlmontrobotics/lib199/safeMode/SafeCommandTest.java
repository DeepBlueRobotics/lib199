package org.carlmontrobotics.lib199.safeMode;

import static org.junit.Assert.*;

import org.junit.Test;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class SafeCommandTest {

    @Test
    public void testSafeCommand() {
        SafeCommand command = new SafeCommand(new RunCommand(() -> {})) {
            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        };

        SafeMode.enable();
        command.schedule();
        CommandScheduler.getInstance().run();
        assertTrue(command.isScheduled());

        SafeMode.disable();
        CommandScheduler.getInstance().run();
        assertFalse(command.isScheduled());
    }

}
