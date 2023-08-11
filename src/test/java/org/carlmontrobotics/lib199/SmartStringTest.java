package org.carlmontrobotics.lib199;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class SmartStringTest {

    SmartString dizzy = new SmartString("Goober", "blue");

    @Test
    public void testGetName() {
        assertEquals(dizzy.getName(), "Goober");
    }

    @Test
    public void testGetUserValue() {
        assertEquals(dizzy.getValue(), "blue", "blue");
    }

    @Test
    public void testSetValue() {
        dizzy.set("splat");
        assertEquals("splat", "splat");
    }

    @Test
    public void testReset() {
        dizzy.reset();
        assertEquals("", "");
    }

}
