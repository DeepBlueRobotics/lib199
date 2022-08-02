package org.carlmontrobotics.lib199;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class LinearInterpolationTest {
    
    @Test
    public void testSimpleLinearInterpolation() {
        LinearInterpolation lInterp = new LinearInterpolation("src/test/resources/LinearInterpolData1.csv");
        assertEquals(3, lInterp.calculate(3), 0.01);

        lInterp = new LinearInterpolation("src/test/resources/LinearInterpolData2.csv");
        assertEquals(3, lInterp.calculate(3), 0.01);
    }

    @Test
    public void testLinearInterpolation() {
        LinearInterpolation lInterp = new LinearInterpolation("src/test/resources/LinearInterpolData3.csv");
        assertEquals(2, lInterp.calculate(2), 0.01);
        assertEquals(3.5, lInterp.calculate(4), 0.01);
    }

}
