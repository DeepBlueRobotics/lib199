package frc.robot.lib.path;

import java.util.function.Supplier;
import java.util.stream.DoubleStream;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public interface DifferentialDriveInterface extends DrivetrainInterface {

    /**
     * Drives in autonomous
     * @param left power to left motor m/s
     * @param right power to right motor m/s
     */
    public void autoDrive(double left, double right);

    /**
     * Gets Differential Drive kinematics
     * @return kinematics
     */
    public DifferentialDriveKinematics getKinematics();

    /**
     * Gets autonomous max volts
     * @return autonomous max volts
     */
    public double getAutoMaxVolt();

    /**
     * Gets characterization values in the form { kVolts, kVels, kAccels }
     * @return characterization values in the form { kVolts, kVels, kAccels }
     */
    public double[][] getCharacterizationValues();

    /**
     * Gets odometry
     * @return Odometry
     */
    public DifferentialDriveOdometry getOdometry();

    /**
     * Sets odometry
     * @param odometry Odometry that will be set
     */
    public void setOdometry(DifferentialDriveOdometry odometry);

    /**
     * Creates Ramsete Controller
     * @return Ramsete Controller
     */
    public default RamseteController createRamsete() {
        return new RamseteController();
    }

    /**
     * Configures Trajectory
     * @param TrajectoryConfig object
     */
    @Override
    public default void configureTrajectory(TrajectoryConfig config) {
        config.setKinematics(getKinematics());

        double[][] charVals = getCharacterizationValues();
        config.addConstraint(new DifferentialDriveVoltageConstraint(
            // new SimpleMotorFeedforward(Utils199.average(charVals[0]), Utils199.average(charVals[1]), Utils199.average(charVals[2])),
            new SimpleMotorFeedforward(DoubleStream.of(charVals[0]).average().getAsDouble(), 
                                        DoubleStream.of(charVals[1]).average().getAsDouble(), 
                                        DoubleStream.of(charVals[2]).average().getAsDouble()),
            getKinematics(), getAutoMaxVolt()));
    }

    /**
     * Creates Ramsete Command
     * @param trajectory Trajectory object
     * @param desiredHeading Desired Heading
     * @return Ramsete Command
     */
    @Override
    public default Command createRamseteCommand(Trajectory trajectory, Supplier<Rotation2d> desiredHeading) {
        return new RamseteCommand(
            trajectory,
            // Call getOdometry in the supplier because the odometry object may be reset when the command is run
            () -> getOdometry().getPoseMeters(),
            createRamsete(),
            getKinematics(),
            this::autoDrive,
            this
        );
    }

    /**
     * Sets odometry based on current gyro angle and pose
     * @param gyroAngle The angle reported by the gyroscope.
     * @param initialPose The starting position of the robot on the field.
     */
    @Override
    public default void setOdometry(Rotation2d gyroAngle, Pose2d initialPose) {
        setOdometry(new DifferentialDriveOdometry(gyroAngle, initialPose));
    }

}
