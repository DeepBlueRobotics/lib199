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

    public void autoDrive(double left, double right);

    public DifferentialDriveKinematics getKinematics();

    public double getAutoMaxVolt();

    // Get the characterization values in the form { kVolts, kVels, kAccels }
    public double[][] getCharacterizationValues();

    public DifferentialDriveOdometry getOdometry();

    public void setOdometry(DifferentialDriveOdometry odometry);

    public default RamseteController createRamsete() {
        return new RamseteController();
    }

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

    @Override
    public default void setOdometry(Rotation2d gyroAngle, Pose2d initialPose) {
        setOdometry(new DifferentialDriveOdometry(gyroAngle, initialPose));
    }

}
