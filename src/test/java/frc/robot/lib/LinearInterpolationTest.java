package frc.robot.lib;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class LinearInterpolationTest {
    
    @Test
    public void testSimpleLinearInterpolation() {
        LinearInterpolation lInterp = new LinearInterpolation("../../test/resources/LinearInterpolData1.csv");
        assertEquals(3, lInterp.calculate(3), 0.01);

        lInterp = new LinearInterpolation("../../test/resources/LinearInterpolData2.csv");
        assertEquals(3, lInterp.calculate(3), 0.01);
    }

    @Test
    public void testLinearInterpolation() {
        LinearInterpolation lInterp = new LinearInterpolation("../../test/resources/LinearInterpolData3.csv");
        assertEquals(2, lInterp.calculate(2), 0.01);
        assertEquals(3.5, lInterp.calculate(4), 0.01);
    }

    @Test
    public void testNegativeSlopeLinearInterpolation() {
        LinearInterpolation lInterp = new LinearInterpolation("../../test/resources/LinearInterpolData4.csv");
        assertEquals(0.75, lInterp.calculate(0.25), 0.01);
        assertEquals(0.25, lInterp.calculate(0.75), 0.01);
    }

    @Test
    public void testPiecewiseLinearInterpolation() {
        LinearInterpolation lInterp = new LinearInterpolation("../../test/resources/LinearInterpolData5.csv");
        assertEquals(1, lInterp.calculate(0), 0.01);
        assertEquals(0.75, lInterp.calculate(0.25), 0.01);
        assertEquals(0.25, lInterp.calculate(0.75), 0.01);
        assertEquals(0, lInterp.calculate(1), 0.01);
        assertEquals(0.25, lInterp.calculate(1.25), 0.01);
        assertEquals(0.75, lInterp.calculate(1.75), 0.01);
        assertEquals(1, lInterp.calculate(2), 0.01);
    }

}
