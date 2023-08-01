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
    
}
