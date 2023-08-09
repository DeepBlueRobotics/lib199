package org.carlmontrobotics.lib199;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class SmartNumberTest
{
    SmartNumber mosh = new SmartNumber("Goober", 4);
    @Test
    public void testGetName()
    {
        assertEquals(mosh.getName(), "Goober");
    }

    @Test
    public void testGetUserValue()
    {
        assertEquals(mosh.getValue(), 4, 0.001);
    }

    @Test
    public void testSetValue()
    {
        mosh.set(2.2);
        assertEquals(mosh.getValue(), 2.2, 2.2);
    }

    @Test
    public void testReset()
    {
        mosh.reset();
        assertEquals(0, 0, 0.001);
    }

}
