package org.carlmontrobotics.lib199.safeMode;

import static org.junit.Assert.*;

import org.carlmontrobotics.lib199.testUtils.TestRules;
import org.junit.ClassRule;
import org.junit.Test;

import edu.wpi.first.wpilibj.GenericHID;

public class SafeJoystickTest {

    @ClassRule
    public static TestRules.InitializeHAL classRule = new TestRules.InitializeHAL();

    @Test
    public void testSafeJoystick() {
        GenericHID normalJoystick = createDummyJoystick(0);
        GenericHID unsafeJoystick1 = createDummyJoystick(1);
        GenericHID unsafeJoystick2 = createDummyJoystick(2);

        GenericHID safeJoystick1 = SafeMode.makeSafe(unsafeJoystick1);

        SafeMode.disableButton(1, 0);
        SafeMode.disableAxis(1, 0);
        SafeMode.scaleAxis(1, 1, 0.5);
        SafeMode.disablePOV(1, 0);
        SafeMode.disablePOV(1, 1, 0);

        SafeMode.disableButton(2, 0);
        SafeMode.disableAxis(2, 0);
        SafeMode.scaleAxis(2, 1, 0.5);
        SafeMode.disablePOV(2, 0);
        SafeMode.disablePOV(2, 1, 0);

        GenericHID safeJoystick2 = SafeMode.makeSafe(unsafeJoystick2);

        SafeMode.disable();

        assertTrue(normalJoystick.getRawButton(0));
        assertTrue(normalJoystick.getRawButtonPressed(0));
        assertTrue(normalJoystick.getRawButtonReleased(0));
        assertTrue(normalJoystick.getRawButton(1));
        assertTrue(normalJoystick.getRawButtonPressed(1));
        assertTrue(normalJoystick.getRawButtonReleased(1));
        assertEquals(1.0, normalJoystick.getRawAxis(0), 0.01);
        assertEquals(1.0, normalJoystick.getRawAxis(1), 0.01);
        assertEquals(1.0, normalJoystick.getRawAxis(2), 0.01);
        assertEquals(0, normalJoystick.getPOV(0));
        assertEquals(90, normalJoystick.getPOV(1));

        assertTrue(safeJoystick1.getRawButton(0));
        assertTrue(safeJoystick1.getRawButtonPressed(0));
        assertTrue(safeJoystick1.getRawButtonReleased(0));
        assertTrue(safeJoystick1.getRawButton(1));
        assertTrue(safeJoystick1.getRawButtonPressed(1));
        assertTrue(safeJoystick1.getRawButtonReleased(1));
        assertEquals(1.0, safeJoystick1.getRawAxis(0), 0.01);
        assertEquals(1.0, safeJoystick1.getRawAxis(1), 0.01);
        assertEquals(1.0, safeJoystick1.getRawAxis(2), 0.01);
        assertEquals(0, safeJoystick1.getPOV(0));
        assertEquals(90, safeJoystick1.getPOV(1));

        assertTrue(safeJoystick2.getRawButton(0));
        assertTrue(safeJoystick2.getRawButtonPressed(0));
        assertTrue(safeJoystick2.getRawButtonReleased(0));
        assertTrue(safeJoystick2.getRawButton(1));
        assertTrue(safeJoystick2.getRawButtonPressed(1));
        assertTrue(safeJoystick2.getRawButtonReleased(1));
        assertEquals(1.0, safeJoystick2.getRawAxis(0), 0.01);
        assertEquals(1.0, safeJoystick2.getRawAxis(1), 0.01);
        assertEquals(1.0, safeJoystick2.getRawAxis(2), 0.01);
        assertEquals(0, safeJoystick2.getPOV(0));
        assertEquals(90, safeJoystick2.getPOV(1));

        SafeMode.enable();

        assertTrue(normalJoystick.getRawButton(0));
        assertTrue(normalJoystick.getRawButtonPressed(0));
        assertTrue(normalJoystick.getRawButtonReleased(0));
        assertTrue(normalJoystick.getRawButton(1));
        assertTrue(normalJoystick.getRawButtonPressed(1));
        assertTrue(normalJoystick.getRawButtonReleased(1));
        assertEquals(1.0, normalJoystick.getRawAxis(0), 0.01);
        assertEquals(1.0, normalJoystick.getRawAxis(1), 0.01);
        assertEquals(1.0, normalJoystick.getRawAxis(2), 0.01);
        assertEquals(0, normalJoystick.getPOV(0));
        assertEquals(90, normalJoystick.getPOV(1));

        assertFalse(safeJoystick1.getRawButton(0));
        assertFalse(safeJoystick1.getRawButtonPressed(0));
        assertFalse(safeJoystick1.getRawButtonReleased(0));
        assertTrue(safeJoystick1.getRawButton(1));
        assertTrue(safeJoystick1.getRawButtonPressed(1));
        assertTrue(safeJoystick1.getRawButtonReleased(1));
        assertEquals(0.0, safeJoystick1.getRawAxis(0), 0.01);
        assertEquals(0.5, safeJoystick1.getRawAxis(1), 0.01);
        assertEquals(1.0, safeJoystick1.getRawAxis(2), 0.01);
        assertEquals(-1, safeJoystick1.getPOV(0));
        assertEquals(90, safeJoystick1.getPOV(1));

        assertFalse(safeJoystick2.getRawButton(0));
        assertFalse(safeJoystick2.getRawButtonPressed(0));
        assertFalse(safeJoystick2.getRawButtonReleased(0));
        assertTrue(safeJoystick2.getRawButton(1));
        assertTrue(safeJoystick2.getRawButtonPressed(1));
        assertTrue(safeJoystick2.getRawButtonReleased(1));
        assertEquals(0.0, safeJoystick2.getRawAxis(0), 0.01);
        assertEquals(0.5, safeJoystick2.getRawAxis(1), 0.01);
        assertEquals(1.0, safeJoystick2.getRawAxis(2), 0.01);
        assertEquals(-1, safeJoystick2.getPOV(0));
        assertEquals(90, safeJoystick2.getPOV(1));
    }

    private static GenericHID createDummyJoystick(int port) {
        return new GenericHID(port) {
            public boolean getRawButton(int button) {
                return true;
            }

            @Override
            public boolean getRawButtonPressed(int button) {
                return true;
            }

            @Override
            public boolean getRawButtonReleased(int button) {
                return true;
            }

            public double getRawAxis(int axis) {
                return 1.0;
            }

            public int getPOV(int pov) {
                return pov == 0 ? 0 : 90;
            }
        };
    }
}
