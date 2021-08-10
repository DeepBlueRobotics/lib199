package frc.robot.lib.swerve;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

public class SwerveMathTest {

    @Test
    public void testCalculateSwerve() {
        {
            SwerveModuleState[] states = SwerveMath.calculateSwerve(0, 0, 0, 0, 0);
            assertEquals(4, states.length);
            for(int i = 0; i < states.length; i++) {
                assertTrue(Double.isNaN(states[i].speedMetersPerSecond));
                assertTrue(Double.isNaN(states[i].angle.getRadians()));
            }
        }
        {
            SwerveModuleState[] states = SwerveMath.calculateSwerve(0, 0, 0, 45, 0, 0);
            assertEquals(4, states.length);
            for(int i = 0; i < states.length; i++) {
                assertTrue(Double.isNaN(states[i].speedMetersPerSecond));
                assertTrue(Double.isNaN(states[i].angle.getRadians()));
            }
        }
        {
            SwerveModuleState[] states = SwerveMath.calculateSwerve(0, 0, 0, 90, 0, 0);
            assertEquals(4, states.length);
            for(int i = 0; i < states.length; i++) {
                assertTrue(Double.isNaN(states[i].speedMetersPerSecond));
                assertTrue(Double.isNaN(states[i].angle.getRadians()));
            }
        }
        assertTestCalculateSwerve(0, 0, 0, 10, 10, 0, 0, 0, 0, 0, 0, 0, 0);
        assertTestCalculateSwerve(1, 0, 0, 10, 10, 1, 90, 1, 90, 1, 90, 1, 90);
        assertTestCalculateSwerve(0, 1, 0, 10, 10, 1, 0, 1, 0, 1, 0, 1, 0);
        assertTestCalculateSwerve(0, 0, 1, 10, 10, 1, 45, 1, -45, 1, -45, 1, 45);
        assertTestCalculateSwerve(1, 1, 0, 10, 10, 1.41, 45, 1.41, 45, 1.41, 45, 1.41, 45);
        assertTestCalculateSwerve(1, 1, 1, 10, 10, 2.41, 45, 1.73, 9.74, 1.73, 80.26, 0.41, 45);
        assertTestCalculateSwerve(-1, -1, 1, 10, 10, 0.41, 45, 1.73, 80.26, 1.73, 9.74, 2.41, 45);
        assertTestCalculateSwerve(1, 1, -1, 10, 10, 0.41, 45, 1.73, 80.26, 1.73, 9.74, 2.41, 45);
        assertTestCalculateSwerve(9, 3, -10, 10, 10, 4.5, -25.35, 16.58, -75.79, 10.25, 10.84, 18.97, 57.93);
        assertTestCalculateSwerve(-7, -4, -1, 10, 10, 9.03, 58.59, 7.86, 53.2, 8.38, 66.87, 7.1, 62.38);
    }

    private void assertTestCalculateSwerve(double forward, double strafe, double rotation, double length, double width, double... expected) {
        SwerveModuleState[] states = SwerveMath.calculateSwerve(forward, strafe, rotation, length, width);
        SwerveModuleState[] headingStates0 = SwerveMath.calculateSwerve(forward, strafe, rotation, 0, length, width);
        double forward45 = forward * Math.cos(Math.toRadians(-45)) + strafe * Math.sin(Math.toRadians(-45));
        double strafe45 = strafe * Math.cos(Math.toRadians(-45)) - forward * Math.sin(Math.toRadians(-45));
        SwerveModuleState[] headingStates45 = SwerveMath.calculateSwerve(forward45, strafe45, rotation, Math.toRadians(45), length, width);
        double forward90 = forward * Math.cos(Math.toRadians(-90)) + strafe * Math.sin(Math.toRadians(-90));
        double strafe90 = strafe * Math.cos(Math.toRadians(-90)) - forward * Math.sin(Math.toRadians(-90));
        SwerveModuleState[] headingStates90 = SwerveMath.calculateSwerve(forward90, strafe90, rotation, Math.toRadians(90), length, width);
        assertEquals(4, states.length);
        assertEquals(4, headingStates0.length);
        assertEquals(4, headingStates45.length);
        assertEquals(4, headingStates90.length);
        for(int i = 0; i < states.length; i++) {
            assertEquals(expected[i*2], states[i].speedMetersPerSecond, 0.01);
            assertEquals(expected[i*2], headingStates0[i].speedMetersPerSecond, 0.01);
            assertEquals(expected[i*2], headingStates45[i].speedMetersPerSecond, 0.01);
            assertEquals(expected[i*2], headingStates90[i].speedMetersPerSecond, 0.01);

            double degExpected = expected[i*2+1];
            degExpected %= 180;
            degExpected += degExpected < 0 ? 180 : 0;
            double degActual = states[i].angle.getDegrees();
            degActual %= 180;
            degActual += degActual < 0 ? 180 : 0;
            double degActual0 = headingStates0[i].angle.getDegrees();
            degActual0 %= 180;
            degActual0 += degActual0 < 0 ? 180 : 0;
            double degActual45 = headingStates45[i].angle.getDegrees();
            degActual45 %= 180;
            degActual45 += degActual45 < 0 ? 180 : 0;
            double degActual90 = headingStates90[i].angle.getDegrees();
            degActual90 %= 180;
            degActual90 += degActual90 < 0 ? 180 : 0;

            assertEquals(degExpected, degActual, 0.01);
            assertEquals(degExpected, degActual0, 0.01);
            assertEquals(degExpected, degActual45, 0.01);
            assertEquals(degExpected, degActual90, 0.01);
        }

        // Inverse Swerve
        assertArrayEquals(new double[] {forward, strafe, rotation}, SwerveMath.inverseSwerve(length, width, states), 0.01);
        assertArrayEquals(new double[] {forward, strafe, rotation}, SwerveMath.inverseSwerve(length, width, 0, headingStates0), 0.01);
        assertArrayEquals(new double[] {forward45, strafe45, rotation}, SwerveMath.inverseSwerve(length, width, Math.toRadians(45), headingStates45), 0.01);
        assertArrayEquals(new double[] {forward90, strafe90, rotation}, SwerveMath.inverseSwerve(length, width, Math.toRadians(90), headingStates90), 0.01);
    }

    @Test
    public void testComputeSetpoints() {
        assertTestComputeSetpoints(0, 0, 1, false, 0);
        assertTestComputeSetpoints(0.5, 2, 4, false, 0.5);
        assertTestComputeSetpoints(0.75, 1, 1, false, 0.75);
        assertTestComputeSetpoints(0.5, 0, 1, true, 0);
        assertTestComputeSetpoints(1, 0.5, 1, true, 0.5);
        assertTestComputeSetpoints(1, 1.5, 1, false, 2);

        assertTestComputeSetpoints(-0.5, 2, 4, false, 0.5);
        assertTestComputeSetpoints(-0.75, 1, 1, false, 1.25);
        assertTestComputeSetpoints(-0.5, 0, 1, true, 0);
        assertTestComputeSetpoints(-1, 0.5, 1, true, -0.5);
        assertTestComputeSetpoints(-1, 1.5, 1, true, 0.5);

        assertTestComputeSetpoints(0.5, -2, 4, false, -0.5);
        assertTestComputeSetpoints(0.75, -1, 1, false, -1.25);
        assertTestComputeSetpoints(1, -0.5, 1, true, -0.5);
        assertTestComputeSetpoints(1, -1.5, 1, true, -0.5);
        assertTestComputeSetpoints(0.2, -0.3, 1, true, -0.3);

        assertTestComputeSetpoints(-0.5, -2, 4, false, -0.5);
        assertTestComputeSetpoints(-0.75, -1, 1, false, -0.75);
        assertTestComputeSetpoints(-1, -0.5, 1, true, -0.5);
        assertTestComputeSetpoints(-1, -1.5, 1, true, -1.5);
    }

    private void assertTestComputeSetpoints(double angle, double encoderPosition, double gearRatio, boolean shouldReverse, double expectedAngle) {
        int shouldReverseM = shouldReverse ? -1 : 1;
        double[] actualN1 = SwerveMath.computeSetpoints(-1, angle, encoderPosition, gearRatio);
        assertArrayEquals(new double[] {-shouldReverseM, expectedAngle}, actualN1, 0.01);
        double[] actual0 = SwerveMath.computeSetpoints(0, angle, encoderPosition, gearRatio);
        assertArrayEquals(new double[] {0, expectedAngle}, actual0, 0.01);
        double[] actual1 = SwerveMath.computeSetpoints(1, angle, encoderPosition, gearRatio);
        assertArrayEquals(new double[] {shouldReverseM, expectedAngle}, actual1, 0.01);
    }

    @Test
    public void testShouldReverse() {
        assertTestShouldReverse(0, 0, 1, false);
        assertTestShouldReverse(0.5, 2, 4, false);
        assertTestShouldReverse(0.75, 1, 1, false);
        assertTestShouldReverse(0.5, 0, 1, true);
        assertTestShouldReverse(1, 0.5, 1, true);
        assertTestShouldReverse(1, 1.5, 1, true);

        assertTestShouldReverse(-0.5, 2, 4, false);
        assertTestShouldReverse(-0.75, 1, 1, false);
        assertTestShouldReverse(-0.5, 0, 1, true);
        assertTestShouldReverse(-1, 0.5, 1, true);
        assertTestShouldReverse(-1, 1.5, 1, true);

        assertTestShouldReverse(0.5, -2, 4, false);
        assertTestShouldReverse(0.75, -1, 1, false);
        assertTestShouldReverse(1, -0.5, 1, false);
        assertTestShouldReverse(1, -1.5, 1, false);
        assertTestShouldReverse(0.2, -0.3, 1, true);

        assertTestShouldReverse(-0.5, -2, 4, false);
        assertTestShouldReverse(-0.75, -1, 1, false);
        assertTestShouldReverse(-1, -0.5, 1, true);
        assertTestShouldReverse(-1, -1.5, 1, true);
    }

    private void assertTestShouldReverse(double angle, double encoderPosition, double gearRatio, boolean shouldReverse) {
        assertFalse(shouldReverse ^ SwerveMath.shouldReverse(angle, encoderPosition, gearRatio));
    }

    @Test
    public void testConvertAngle() {
        assertTestConvertAngle(-1, 0, 0);
        assertTestConvertAngle(-0.8, 0, 0.2);
        assertTestConvertAngle(-0.5, 0, -0.5);
        assertTestConvertAngle(-0.25, 0, -0.25);
        assertTestConvertAngle(0, 0, 0);
        assertTestConvertAngle(0.25, 0, 0.25);
        assertTestConvertAngle(0.5, 0, 0.5);
        assertTestConvertAngle(0.8, 0, -0.2);
        assertTestConvertAngle(1, 0, 0);

        assertTestConvertAngle(-1, -0.5, -1);
        assertTestConvertAngle(-0.5, -0.5, -0.5);
        assertTestConvertAngle(0, -0.5, 0);
        assertTestConvertAngle(0.5, -0.5, -0.5);
        assertTestConvertAngle(1, -0.5, 0);

        assertTestConvertAngle(-1, 0.5, 0);
        assertTestConvertAngle(-0.5, 0.5, 0.5);
        assertTestConvertAngle(0, 0.5, 0);
        assertTestConvertAngle(0.5, 0.5, 0.5);
        assertTestConvertAngle(1, 0.5, 1);

        assertTestConvertAngle(-1, 1, 1);
        assertTestConvertAngle(1, -1, -1);
    }


    private void assertTestConvertAngle(double angle, double encoderPosition, double result) {
        assertEquals(result, SwerveMath.convertAngle(angle, encoderPosition, 1), 0.01);
        assertEquals(result, SwerveMath.convertAngle(angle, encoderPosition*2, 2), 0.01);
    }

}
