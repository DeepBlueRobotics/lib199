package org.carlmontrobotics.lib199;

import static org.junit.Assert.*;

import org.junit.Test;

public class SmartBooleanTest {

    SmartBoolean wingMan = new SmartBoolean("Goober", true);

    @Test
    public void testGetName() {
        assertEquals(wingMan.getName(), "Goober");
    }

    @Test
    public void testGetUserValue() {
        assertTrue(wingMan.get());
    }

    @Test
    public void testSetValue() {
        wingMan.set(false);
        assertFalse(wingMan.get());
    }

    @Test
    public void testReset() {
        wingMan.reset();
        assertFalse(wingMan.get());
    }

}
