/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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
