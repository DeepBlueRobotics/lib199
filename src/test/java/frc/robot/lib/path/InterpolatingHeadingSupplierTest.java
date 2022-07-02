package frc.robot.lib.path;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class InterpolatingHeadingSupplierTest {

    @Test
    public void testCalculateFinalAngle() {
        assertEquals(0, InterpolatingHeadingSupplier.calculateFinalAngle(0, 0), 0.01);
        assertEquals(30, InterpolatingHeadingSupplier.calculateFinalAngle(0, 30), 0.01);
        assertEquals(60, InterpolatingHeadingSupplier.calculateFinalAngle(0, 60), 0.01);
        assertEquals(90, InterpolatingHeadingSupplier.calculateFinalAngle(0, 90), 0.01);
        assertEquals(120, InterpolatingHeadingSupplier.calculateFinalAngle(0, 120), 0.01);
        assertEquals(150, InterpolatingHeadingSupplier.calculateFinalAngle(0, 150), 0.01);
        assertEquals(180, Math.abs(InterpolatingHeadingSupplier.calculateFinalAngle(0, 180)), 0.01);
        assertEquals(-150, InterpolatingHeadingSupplier.calculateFinalAngle(0, 210), 0.01);
        assertEquals(-120, InterpolatingHeadingSupplier.calculateFinalAngle(0, 240), 0.01);
        assertEquals(-90, InterpolatingHeadingSupplier.calculateFinalAngle(0, 270), 0.01);
        assertEquals(-60, InterpolatingHeadingSupplier.calculateFinalAngle(0, 300), 0.01);
        assertEquals(-30, InterpolatingHeadingSupplier.calculateFinalAngle(0, 330), 0.01);
        assertEquals(0, InterpolatingHeadingSupplier.calculateFinalAngle(0, 360), 0.01);

        assertEquals(0, InterpolatingHeadingSupplier.calculateFinalAngle(90, 0), 0.01);
        assertEquals(30, InterpolatingHeadingSupplier.calculateFinalAngle(90, 30), 0.01);
        assertEquals(60, InterpolatingHeadingSupplier.calculateFinalAngle(90, 60), 0.01);
        assertEquals(90, InterpolatingHeadingSupplier.calculateFinalAngle(90, 90), 0.01);
        assertEquals(120, InterpolatingHeadingSupplier.calculateFinalAngle(90, 120), 0.01);
        assertEquals(150, InterpolatingHeadingSupplier.calculateFinalAngle(90, 150), 0.01);
        assertEquals(180, InterpolatingHeadingSupplier.calculateFinalAngle(90, 180), 0.01);
        assertEquals(210, InterpolatingHeadingSupplier.calculateFinalAngle(90, 210), 0.01);
        assertEquals(240, InterpolatingHeadingSupplier.calculateFinalAngle(90, 240), 0.01);
        try {
            assertEquals(270, InterpolatingHeadingSupplier.calculateFinalAngle(90, 270), 0.01);
        } catch(AssertionError e) {
            assertEquals(-90, InterpolatingHeadingSupplier.calculateFinalAngle(90, 270), 0.01);
        }
        assertEquals(-60, InterpolatingHeadingSupplier.calculateFinalAngle(90, 300), 0.01);
        assertEquals(-30, InterpolatingHeadingSupplier.calculateFinalAngle(90, 330), 0.01);
        assertEquals(0, InterpolatingHeadingSupplier.calculateFinalAngle(90, 360), 0.01);

        assertEquals(360, InterpolatingHeadingSupplier.calculateFinalAngle(270, 0), 0.01);
        assertEquals(390, InterpolatingHeadingSupplier.calculateFinalAngle(270, 30), 0.01);
        assertEquals(420, InterpolatingHeadingSupplier.calculateFinalAngle(270, 60), 0.01);
        try {
            assertEquals(90, InterpolatingHeadingSupplier.calculateFinalAngle(270, 90), 0.01);
        } catch(AssertionError e) {
            assertEquals(450, InterpolatingHeadingSupplier.calculateFinalAngle(270, 90), 0.01);
        }
        assertEquals(120, InterpolatingHeadingSupplier.calculateFinalAngle(270, 120), 0.01);
        assertEquals(150, InterpolatingHeadingSupplier.calculateFinalAngle(270, 150), 0.01);
        assertEquals(180, InterpolatingHeadingSupplier.calculateFinalAngle(270, 180), 0.01);
        assertEquals(210, InterpolatingHeadingSupplier.calculateFinalAngle(270, 210), 0.01);
        assertEquals(240, InterpolatingHeadingSupplier.calculateFinalAngle(270, 240), 0.01);
        assertEquals(270, InterpolatingHeadingSupplier.calculateFinalAngle(270, 270), 0.01);
        assertEquals(300, InterpolatingHeadingSupplier.calculateFinalAngle(270, 300), 0.01);
        assertEquals(330, InterpolatingHeadingSupplier.calculateFinalAngle(270, 330), 0.01);
        assertEquals(360, InterpolatingHeadingSupplier.calculateFinalAngle(270, 360), 0.01);
    }

}
