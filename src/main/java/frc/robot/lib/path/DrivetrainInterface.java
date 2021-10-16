package frc.robot.lib.path;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface DrivetrainInterface extends Subsystem {

    public void configureTrajectory(TrajectoryConfig config);

    public Command createRamseteCommand(Trajectory trajectory, Supplier<Rotation2d> desiredHeading);

    public double getAutoMaxAccel();

    public double getAutoMaxSpeed();

    public double getHeading();

    public void setOdometry(Rotation2d gyroAngle, Pose2d initialPose);

    public void stop();

}