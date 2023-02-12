package org.carlmontrobotics.lib199.path;

import java.util.Arrays;
import java.util.HashMap;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public interface SwerveDriveInterface extends DrivetrainInterface {

    /**
     * Drives given swerve module states
     * 
     * @param swerveModuleStates Array of Swerve Module states
     */
    public void drive(SwerveModuleState[] swerveModuleStates);

    /**
     * Gets Swerve Drive kinematics
     * 
     * @return kinematics
     */
    public SwerveDriveKinematics getKinematics();

    /**
     * Retrieves the PID constants which will be used for path following in the form
     * { xPID, yPID, zPID } The values in these arrays will be read as P, I, D All
     * three xPID, yPID, zPID and P, I, D values must be included
     * 
     * @return PID constnants
     */
    public double[][] getPIDConstants();

    /**
     * Sets odometry based on current kinematics, gyro angle, pose
     * 
     * @param odometry current kinematics, gyro angle, pose
     */
    public void setOdometry(SwerveDriveOdometry odometry);

    /**
     * @return The current positions of the swerve modules
     */
    public SwerveModulePosition[] getModulePositions();

    /**
     * Configures the constants for generating a trajectory
     * WARNING: THIS METHOD IS NOT CALLED IF USING PPRobotPath!
     * 
     * @param path The configuration for generating a trajectory
     */
    @Override
    public default void configureAutoPath(RobotPath path) {
        // This performs the same action as setKinematics but with our maxSpeed
        TrajectoryConfig config = path.getTrajectoryConfig();
        config.addConstraint(new SwerveDriveKinematicsConstraint(getKinematics(), path.getMaxAccelMps2()));
    }

    /**
     * Constructs a new SwerveControllerCommand that, when executed, will follow the
     * provided trajectory.
     * 
     * @param trajectory     The trajectory to follow.
     * @param desiredHeading A function that supplies the robot pose - use one of
     *                       the odometry classes to provide this.
     * @return SwerveControllerCommand
     */
    @Override
    public default Command createAutoCommand(Trajectory trajectory, Supplier<Rotation2d> desiredHeading) {
        double[][] pidConstants = getPIDConstants();
        double[] xPID = pidConstants[0];
        PIDController xController = new PIDController(xPID[0], xPID[1], xPID[2]);
        double[] yPID = pidConstants[1];
        PIDController yController = new PIDController(yPID[0], yPID[1], yPID[2]);
        double[] thetaPID = pidConstants[2];
        ProfiledPIDController thetaController = new ProfiledPIDController(thetaPID[0], thetaPID[1], thetaPID[2],
                new Constraints(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        return new SwerveControllerCommand(trajectory, this::getPose, getKinematics(), xController, yController, thetaController, desiredHeading, this::drive, this);
    }

    /**
     * Constructs a new PPSwerveControllerCommand that, when executed, will follow the
     * provided trajectory.
     * 
     * @param trajectory The trajectory to follow.
     * @param eventMap   Map of event marker names to the commands that should run when reaching that marker.
     *                   This SHOULD NOT contain any commands requiring Drivetrain, or it will be interrupted
     * @return PPSwerveControllerCommand
     */
    public default Command createPPAutoCommand(PathPlannerTrajectory trajectory, HashMap<String, Command> eventMap) {
        Subsystem[] requirements = eventMap.values().stream().flatMap(command -> command.getRequirements().stream())
                .toArray(Subsystem[]::new);
        requirements = Arrays.copyOf(requirements, requirements.length + 1);
        requirements[requirements.length - 1] = this;
        requirements = Arrays.stream(requirements).distinct().toArray(Subsystem[]::new);
        PIDController[] pidControllers = Arrays.stream(getPIDConstants()).map(constants -> new PIDController(constants[0], constants[1], constants[2])).toArray(PIDController[]::new);
        pidControllers[2].enableContinuousInput(-Math.PI, Math.PI);
        return new PPSwerveControllerCommand(trajectory, this::getPose, getKinematics(), pidControllers[0], pidControllers[1], pidControllers[2], this::drive, true, requirements);
    }

}
