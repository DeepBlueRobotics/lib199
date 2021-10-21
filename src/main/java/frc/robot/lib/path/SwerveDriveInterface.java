package frc.robot.lib.path;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public interface SwerveDriveInterface extends DrivetrainInterface {

    /**
     * Drives in autonomous given swerve module states
     * @param swerveModuleStates Array of Swerve Module states
     */
    public void autoDrive(SwerveModuleState[] swerveModuleStates);

    /**
     * Gets Swerve Drive kinematics
     * @return kinematics
     */
    public SwerveDriveKinematics getKinematics();

    /**
     * Gets Swerve Drive odometry
     * @return odometry
     */
    public SwerveDriveOdometry getOdometry();

    /**
     * Retrieves the PID constants which will be used for path following in the form { xPID, yPID, zPID }
     * The values in these arrays will be read as P, I, D
     * All three xPID, yPID, zPID and P, I, D values must be included
     * @return PID constnants
     */
    public double[][] getPIDConstants();

    /**
     * Sets odometry based on current kinematics, gyro angle, pose
     * @param odometry current kinematics, gyro angle, pose
     */
    public void setOdometry(SwerveDriveOdometry odometry);

    /**
     * Configures the constants for generating a trajectory
     * @param path The configuration for generating a trajectory
     */
    @Override
    public default void configureAutoPath(RobotPath path) {
        // This performs the same action as setKinematics but with our maxSpeed
        TrajectoryConfig config = path.getTrajectoryConfig();
        config.addConstraint(new SwerveDriveKinematicsConstraint(getKinematics(), path.getMaxAccelMps2()));
    }

    /**
     * Constructs a new SwerveControllerCommand that, when executed, will follow the provided trajectory.
     * @param trajectory The trajectory to follow.
     * @param desiredHeading A function that supplies the robot pose - use one of the odometry classes to provide this.
     * @return SwerveControllerCommand
     */
    @Override
    public default Command createAutoCommand(Trajectory trajectory, Supplier<Rotation2d> desiredHeading) {
        double[][] pidConstants = getPIDConstants();
        double[] xPID = pidConstants[0];
        PIDController xController = new PIDController(xPID[0],
                                                         xPID[1],
                                                         xPID[2]);
        double[] yPID = pidConstants[1];
        PIDController yController = new PIDController(yPID[0],
                                                         yPID[1],
                                                         yPID[2]);
        double[] thetaPID = pidConstants[2];
        ProfiledPIDController thetaController = new ProfiledPIDController(thetaPID[0],
                                                                          thetaPID[1],
                                                                          thetaPID[2],
                                                                          new Constraints(Double.POSITIVE_INFINITY,
                                                                                          Double.POSITIVE_INFINITY));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        return new SwerveControllerCommand(
            trajectory,
            // Call getOdometry in the supplier because the odometry object may be reset when the command is run
            () -> getOdometry().getPoseMeters(),
            getKinematics(),
            xController,
            yController,
            thetaController,
            desiredHeading,
            this::autoDrive,
            this
        );
    }

    /**
    * Sets odometry based on current kinematics, gyro angle, and pose
    * @param gyroAngle The angle reported by the gyroscope.
    * @param initialPose The starting position of the robot on the field.
    */
    @Override
    public default void setOdometry(Rotation2d gyroAngle, Pose2d initialPose) {
        setOdometry(new SwerveDriveOdometry(getKinematics(), gyroAngle, initialPose));
    }

}
