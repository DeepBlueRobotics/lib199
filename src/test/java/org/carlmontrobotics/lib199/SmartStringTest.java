package org.carlmontrobotics.lib199;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class SmartStringTest {

    SmartString dizzy = new SmartString("Gargle", "blue");

    @Test
    public void testGetName() {
        assertEquals(dizzy.getName(), "Gargle");
    }

    @Test
    public void testGetUserValue() {
        assertEquals(dizzy.getValue(), "blue");
    }

    @Test
    public void testSetValue() {
        dizzy.set("splat");
        assertEquals(dizzy.getName(), "splat");
    }

    @Test
    public void testReset() {
        dizzy.reset();
        assertEquals(dizzy.getName(), "");
    }

}
