package frc.robot.lib;

import java.util.StringJoiner;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * Modified from {@link edu.wpi.first.wpilibj.drive.KilloughDrive}
 */
public class OmniDrive extends RobotDriveBase implements Sendable, AutoCloseable {
public static final double kDefaultFrontMotorAngle = 45.0;
  public static final double kDefaultLeftMotorAngle = 135.0;
  public static final double kDefaultRightMotorAngle = 225.0;
  public static final double kDefaultBackMotorAngle = 315.0;

  private static int instances;

  private SpeedController m_frontMotor;
  private SpeedController m_leftMotor;
  private SpeedController m_rightMotor;
  private SpeedController m_backMotor;

  private Vector2d m_frontVec;
  private Vector2d m_leftVec;
  private Vector2d m_rightVec;
  private Vector2d m_backVec;

  public OmniDrive(SpeedController frontMotor, SpeedController leftMotor, SpeedController rightMotor,
                       SpeedController backMotor) {
    this(frontMotor, leftMotor, rightMotor, backMotor, kDefaultFrontMotorAngle, kDefaultLeftMotorAngle, kDefaultRightMotorAngle,
        kDefaultBackMotorAngle);
  }

  public OmniDrive(SpeedController frontMotor,SpeedController leftMotor, SpeedController rightMotor,
                       SpeedController backMotor, double frontMotorAngle, double leftMotorAngle, double rightMotorAngle,
                       double backMotorAngle) {
    verify(frontMotor, leftMotor, rightMotor, backMotor);
    m_frontMotor = frontMotor;
    m_leftMotor = leftMotor;
    m_rightMotor = rightMotor;
    m_backMotor = backMotor;
    m_frontVec = new Vector2d(Math.cos(frontMotorAngle * (Math.PI / 180.0)),
                             Math.sin(frontMotorAngle * (Math.PI / 180.0)));
    m_leftVec = new Vector2d(Math.cos(leftMotorAngle * (Math.PI / 180.0)),
                             Math.sin(leftMotorAngle * (Math.PI / 180.0)));
    m_rightVec = new Vector2d(Math.cos(rightMotorAngle * (Math.PI / 180.0)),
                              Math.sin(rightMotorAngle * (Math.PI / 180.0)));
    m_backVec = new Vector2d(Math.cos(backMotorAngle * (Math.PI / 180.0)),
                             Math.sin(backMotorAngle * (Math.PI / 180.0)));
    SendableRegistry.addChild(this, m_frontMotor);
    SendableRegistry.addChild(this, m_leftMotor);
    SendableRegistry.addChild(this, m_rightMotor);
    SendableRegistry.addChild(this, m_backMotor);
    instances++;
    SendableRegistry.addLW(this, "OmniDrive", instances);
  }

  @Override
  public void close() {
    SendableRegistry.remove(this);
  }
  private void verify(SpeedController frontMotor, SpeedController leftMotor, SpeedController rightMotor,
                      SpeedController backMotor) {
    if (frontMotor != null && leftMotor != null && rightMotor != null && backMotor != null) {
      return;
    }
    StringJoiner joiner = new StringJoiner(", ");
    if (frontMotor == null) {
      joiner.add("frontMotor");
    }
    if (leftMotor == null) {
      joiner.add("leftMotor");
    }
    if (rightMotor == null) {
      joiner.add("rightMotor");
    }
    if (backMotor == null) {
      joiner.add("backMotor");
    }
    throw new NullPointerException(joiner.toString());
  }
  
  public void driveCartesian(double ySpeed, double xSpeed, double zRotation) {
    driveCartesian(ySpeed, xSpeed, zRotation, 0.0);
  }

  public void driveCartesian(double ySpeed, double xSpeed, double zRotation,
                             double gyroAngle) {

    ySpeed = MathUtil.clamp(ySpeed, -1.0, 1.0);
    ySpeed = applyDeadband(ySpeed, m_deadband);

    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    xSpeed = applyDeadband(xSpeed, m_deadband);

    // Compensate for gyro angle.
    Vector2d input = new Vector2d(ySpeed, xSpeed);
    input.rotate(-gyroAngle);

    double[] wheelSpeeds = new double[4];
    wheelSpeeds[0] = input.scalarProject(m_frontVec) + zRotation;
    wheelSpeeds[1] = input.scalarProject(m_leftVec) + zRotation;
    wheelSpeeds[2] = input.scalarProject(m_rightVec) + zRotation;
    wheelSpeeds[3] = input.scalarProject(m_backVec) + zRotation;

    normalize(wheelSpeeds);

    m_frontMotor.set(wheelSpeeds[0] * m_maxOutput);
    m_leftMotor.set(wheelSpeeds[1] * m_maxOutput);
    m_rightMotor.set(wheelSpeeds[2] * m_maxOutput);
    m_backMotor.set(wheelSpeeds[3] * m_maxOutput);

    feed();
  }

  public void drivePolar(double magnitude, double angle, double zRotation) {
    driveCartesian(magnitude * Math.sin(angle * (Math.PI / 180.0)),
                   magnitude * Math.cos(angle * (Math.PI / 180.0)), zRotation, 0.0);
  }

  @Override
  public void stopMotor() {
    m_rightMotor.stopMotor();
    m_leftMotor.stopMotor();
    m_rightMotor.stopMotor();
    m_backMotor.stopMotor();
    feed();
  }

  @Override
  public String getDescription() {
    return "OmniDrive";
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("KilloughDrive");
    builder.setActuator(true);
    builder.setSafeState(this::stopMotor);
    builder.addDoubleProperty("Front Motor Speed", m_frontMotor::get, m_frontMotor::set);
    builder.addDoubleProperty("Left Motor Speed", m_leftMotor::get, m_leftMotor::set);
    builder.addDoubleProperty("Right Motor Speed", m_rightMotor::get, m_rightMotor::set);
    builder.addDoubleProperty("Back Motor Speed", m_backMotor::get, m_backMotor::set);
  }
}
