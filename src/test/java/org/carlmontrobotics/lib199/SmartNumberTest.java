package org.carlmontrobotics.lib199;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class SmartNumberTest {

    SmartNumber mosh = new SmartNumber("wooper", 4);

    @Test
    public void testGetName() {
        assertEquals(mosh.getName(), "wooper");
    }

    @Test
    public void testGetUserValue() {
        assertEquals(mosh.get(), 2.2, 0.001);
    }

    @Test
    public void testSetValue() {
        mosh.set(2.2);
        assertEquals(mosh.get(), 2.2, 2.2);
    }

    @Test
    public void testReset() {
        mosh.reset();
        assertEquals(mosh.get(), 4, 0.001);
    }

}
