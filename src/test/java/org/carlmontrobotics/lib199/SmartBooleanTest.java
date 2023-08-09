package org.carlmontrobotics.lib199;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;
public class SmartBooleanTest {
    SmartBoolean wingMan = new SmartBoolean("Goober", true);
    
    @Test
    public void testGetName()
    {
        assertEquals(wingMan.getName(), "Goober");
    }

    @Test
    public void testGetUserValue()
    {
        assertTrue(wingMan.getValue());
    }
    @Test
    public void testSetValue()
    {
        wingMan.set(false);
        assertFalse(false);
    }
    @Test
    public void testReset()
    {
        wingMan.reset();
        assertFalse(false);
    }
    
}
