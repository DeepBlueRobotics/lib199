package frc.robot.lib.path;

import java.util.ArrayList;
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
import edu.wpi.first.wpilibj.trajectory.constraint.EllipticalRegionConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public interface SwerveDriveInterface extends DrivetrainInterface {

    public void autoDrive(SwerveModuleState[] swerveModuleStates);

    public SwerveDriveKinematics getKinematics();

    public SwerveDriveOdometry getOdometry();

    // Retrieves the PID constants which will be used for path following in the form { xPID, yPID, zPID }
    // The values in these arrays will be read as P, I, D
    // All three xPID, yPID, zPID and P, I, D values must be included
    public double[][] getPIDConstants();

    public void setOdometry(SwerveDriveOdometry odometry);

    // Retrieves the end velocity which will be used when configuring the next path
    // (This will be done when the RobotPath constructor is called. NOT when the path is run)
    public default double getEndVelocityForNextPath() {
        return 0;
    }

    // Retrieves the region constraints which will be used when configuring the next path
    // (This will be done when the RobotPath constructor is called. NOT when the path is run)
    public default ArrayList<EllipticalRegionConstraint> getRegionConstraintsForNextPath() {
        return new ArrayList<>();
    }

    @Override
    public default void configureTrajectory(TrajectoryConfig config) {
        // This performs the same action as setKinematics but with our maxSpeed
        config.addConstraint(new SwerveDriveKinematicsConstraint(getKinematics(), getAutoMaxSpeed()));

        // Ensure that the robot turns slowly around tight turns and doesn't slip
        config.addConstraints(getRegionConstraintsForNextPath());
        config.setEndVelocity(getEndVelocityForNextPath());
    }

    @Override
    public default Command createRamseteCommand(Trajectory trajectory, Supplier<Rotation2d> desiredHeading) {
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

    @Override
    public default void setOdometry(Rotation2d gyroAngle, Pose2d initialPose) {
        setOdometry(new SwerveDriveOdometry(getKinematics(), gyroAngle, initialPose));
    }

}
