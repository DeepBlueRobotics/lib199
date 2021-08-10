package frc.robot.lib.swerve;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint.MinMax;

public class SwerveDriveVoltageConstraintTest {

    public static void main(String[] args) {
        new SwerveDriveVoltageConstraintTest().testMinMaxAcceleration();
    }
    
    private final SwerveDriveVoltageConstraint constraint = new SwerveDriveVoltageConstraint(
        new SimpleMotorFeedforward(1, 1, 1),
        new SwerveDriveKinematics(
            new Translation2d(10, 10),
            new Translation2d(10, -10),
            new Translation2d(-10, -10),
            new Translation2d(-10, 10)
        ), 12, 10);

    @Test
    public void testMaxVelocity() {
        assertEquals(Double.POSITIVE_INFINITY, constraint.getMaxVelocityMetersPerSecond(new Pose2d(), 1, 1), 0);
    }

    @Test
    public void testMinMaxAcceleration() {
        assertMinMaxAccelerationTest(0, 0, 0, 0, 0, -12, 12);
        assertMinMaxAccelerationTest(0, 0, 0, 0, 1, -14, 10);

        assertMinMaxAccelerationTest(1, 0, 0, 0, 1, -14, 10);
        assertMinMaxAccelerationTest(0, 1, 0, 0, 1, -14, 10);
        assertMinMaxAccelerationTest(1, 1, 0, 0, 1, -14, 10);

        assertMinMaxAccelerationTest(-1, 0, 0, 0, 1, -14, 10);
        assertMinMaxAccelerationTest(0, -1, 0, 0, 1, -14, 10);
        assertMinMaxAccelerationTest(-1, -1, 0, 0, 1, -14, 10);

        assertMinMaxAccelerationTest(1, 1, 0, 0, -1, -14, 10);

        assertMinMaxAccelerationTest(1, 1, 1, 0, 1, -14, 10);
        assertMinMaxAccelerationTest(1, 1, -1, 0, 1, -14, 10);

        assertMinMaxAccelerationTest(1, 1, 1, 1, 1, -6.61, -0.64);
        assertMinMaxAccelerationTest(-1, -1, -1, -1, -1, -4.41, -0.97);

        assertMinMaxAccelerationTest(1, 1, 1, 1, 0, -2, 2);
        assertMinMaxAccelerationTest(1, 1, 1, 0.1, 1, -27.97, 5.84);

        assertMinMaxAccelerationTest(1, 2, 3, 4, 5, -15.38, -13.12);
        assertMinMaxAccelerationTest(5, 4, 3, 2, 1, -4.51, -1.64);
        assertMinMaxAccelerationTest(4, 9, 7, 9, 10, -29.04, -27.6);
        assertMinMaxAccelerationTest(10, 6, 2, 7, 1, -3.27, -2.46);
    }

    private void assertMinMaxAccelerationTest(double px, double py, double pt, double c, double v, double min, double max) {
        assertMinMaxEquals(min, max, constraint.getMinMaxAccelerationMetersPerSecondSq(pose(px, py, pt), c, v));
    }
    
    private Pose2d pose(double x, double y, double deg) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(deg));
    }

    private void assertMinMaxEquals(double min, double max, MinMax minMax) {
        assertEquals(min, minMax.minAccelerationMetersPerSecondSq, 0.01);
        assertEquals(max, minMax.maxAccelerationMetersPerSecondSq, 0.01);
    }

}
