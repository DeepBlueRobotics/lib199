package org.carlmontrobotics.lib199.path;

import java.util.function.Supplier;
import java.util.stream.DoubleStream;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public interface DifferentialDriveInterface extends DrivetrainInterface {

    /**
     * Drives
     * 
     * @param left  power to left motor m/s
     * @param right power to right motor m/s
     */
    public void drive(double left, double right);

    /**
     * Gets Differential Drive kinematics
     * 
     * @return kinematics
     */
    public DifferentialDriveKinematics getKinematics();

    /**
     * Gets max volts
     * 
     * @return max volts
     */
    public double getMaxVolt();

    /**
     * Gets characterization values in the form { kVolts, kVels, kAccels }
     * 
     * @return characterization values in the form { kVolts, kVels, kAccels }
     */
    public double[][] getCharacterizationValues();

    /**
     * @return The left encoder position in meters
     */
    public double getLeftEncoderPosition();

    /**
     * @return The right encoder position in meters
     */
    public double getRightEncoderPosition();

    /**
     * Creates Ramsete Controller
     * 
     * @return Ramsete Controller
     */
    public default RamseteController createRamsete() {
        return new RamseteController();
    }

    /**
     * Configures Trajectory
     * 
     * @param path RobotPath object
     */
    @Override
    public default void configureAutoPath(RobotPath path) {
        TrajectoryConfig config = path.getTrajectoryConfig();
        config.setKinematics(getKinematics());

        double[][] charVals = getCharacterizationValues();
        config.addConstraint(new DifferentialDriveVoltageConstraint(
                // new SimpleMotorFeedforward(Utils199.average(charVals[0]),
                // Utils199.average(charVals[1]), Utils199.average(charVals[2])),
                new SimpleMotorFeedforward(DoubleStream.of(charVals[0]).average().getAsDouble(),
                        DoubleStream.of(charVals[1]).average().getAsDouble(),
                        DoubleStream.of(charVals[2]).average().getAsDouble()),
                getKinematics(), getMaxVolt()));
    }

    /**
     * Creates Ramsete Command
     * 
     * @param trajectory     Trajectory object
     * @param desiredHeading Desired Heading
     * @return Ramsete Command
     */
    @Override
    public default Command createAutoCommand(Trajectory trajectory, Supplier<Rotation2d> desiredHeading) {
        //Ramsetecommand is deprecated but there's no equivalent...
        return new RamseteCommand(trajectory, this::getPose, createRamsete(), getKinematics(), this::drive, this);
    }

}
