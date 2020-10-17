/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot {
  private RobotContainer robotContainer;
  private com.cyberbotics.webots.controller.Robot robot;
  private int timeStep;
  private SequentialCommandGroup fullRoutine;

  @Override
  public void robotInit() {
    robot = new com.cyberbotics.webots.controller.Robot();
    robotContainer = new RobotContainer(robot);
    timeStep = (int) Math.round(robot.getBasicTimeStep());
    // Make sure to remove the robot when the WPIlib simulation ends
    Runtime.getRuntime().addShutdownHook(new Thread(robot::delete));

    // Load a path to follow and create a RamseteCommand for that path
    try {
      String trajectoryName = "Bruh";
      Path pathToTrajectoryJson = Paths.get(System.getProperty("user.dir") + "/Pathweaver/output/" + trajectoryName + ".wpilib.json");
      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(pathToTrajectoryJson);
      Drivetrain dt = robotContainer.getDrivetrain();
      
      RamseteCommand followPath = new RamseteCommand(trajectory, dt::getPose, new RamseteController(), dt.getKinematics(), dt::tankDriveDirect, dt);
      InstantCommand loadOdometry = new InstantCommand(() -> dt.loadOdometry(trajectory.getInitialPose(), dt.getHeading()));
      fullRoutine = loadOdometry.andThen(followPath, new InstantCommand(() -> dt.tankDrive(0, 0), dt));
    } catch (IOException io) {
      io.printStackTrace();
      fullRoutine = null;
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (RobotBase.isSimulation()) robot.step(timeStep);
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    if (fullRoutine != null) {
      fullRoutine.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
  
  @Override
  public void testPeriodic() {
  }
}
