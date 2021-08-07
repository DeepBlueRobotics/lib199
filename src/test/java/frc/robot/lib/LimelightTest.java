package frc.robot.lib;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightTest {
    
    @Test
    public void testDetermineMountingAngle() {
        Limelight lime = new Limelight();
        // Set ty to 0
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").setDouble(0.0);

        assertEquals(45.0, lime.determineMountingAngle(1, 1, 0), 0.01);
        assertEquals(45.0, lime.determineMountingAngle(1, 2, 1), 0.01);
        assertEquals(Math.toDegrees(Math.atan(2)), lime.determineMountingAngle(1, 2, 0), 0.01);

        // A distance of 0 should return 90 deg
        assertEquals(90.0, lime.determineMountingAngle(0, 2, 1), 0.01);

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").setDouble(45.0);
        assertEquals(0.0, lime.determineMountingAngle(1, 1, 0), 0.01);
    }

    @Test
    public void testDetermineObjectDist() {
        Limelight lime = new Limelight();
        // Set tx and ty to 0
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").setDouble(0.0);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").setDouble(0.0);

        assertArrayEquals(new double[] {1, 0}, lime.determineObjectDist(2, 1, 45.0), 0.01);
        assertArrayEquals(new double[] {2, 0}, lime.determineObjectDist(2, 0, 45.0), 0.01);
        assertArrayEquals(new double[] {0, 0}, lime.determineObjectDist(2, 1, 90.0), 0.01);

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").setDouble(45.0);
        assertArrayEquals(new double[] {1, 0}, lime.determineObjectDist(2, 1, 0), 0.01);

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").setDouble(45.0);
        assertArrayEquals(new double[] {1, Math.sqrt(2)}, lime.determineObjectDist(2, 1, 0), 0.01);
    }

}
