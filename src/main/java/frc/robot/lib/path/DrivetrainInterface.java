package frc.robot.lib.path;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface DrivetrainInterface extends Subsystem {

    /**
     * Configures the constants for generating a trajectory
     * @param config The configuration for generating a trajectory
     */
    public void configureTrajectory(TrajectoryConfig config);

    /**
     * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
     * @param trajectory The trajectory to follow.
     * @param desiredHeading A function that supplies the robot pose - use one of the odometry classes to provide this.
     * @return RamseteCommand
     */
    public Command createRamseteCommand(Trajectory trajectory, Supplier<Rotation2d> desiredHeading);

    /**
     * Gets the Autonomous max acceleration in m/s^2
     * @return max acceleration in m/s^2
     */
    public double getAutoMaxAccelMps2();

    /**
     * Gets the Autonomous max speed in m/s
     * @return max speed in m/s
     */
    public double getAutoMaxSpeedMps();

    /**
     * Gets the current heading in degrees
     * @return current heading in degrees
     */
    public double getHeadingDeg();

    /**
     * Sets odometry based on current gyro angle and pose
     * @param gyroAngle The angle reported by the gyroscope.
     * @param initialPose The starting position of the robot on the field.
     */
    public void setOdometry(Rotation2d gyroAngle, Pose2d initialPose);

    /**
     * Stops the drivetrain
     */
    public void stop();

}
