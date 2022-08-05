package org.carlmontrobotics.lib199;

import java.util.StringJoiner;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * Modified from {@link edu.wpi.first.wpilibj.drive.KilloughDrive}
 */
public class OmniDrive extends RobotDriveBase implements Sendable, AutoCloseable {
  public static final double kDefaultFrontLeftMotorAngle = 315.0;
  public static final double kDefaultFrontRightMotorAngle = 45.0;
  public static final double kDefaultBackLeftMotorAngle = 225.0;
  public static final double kDefaultBackRightMotorAngle = 135.0;

  private static int instances;

  private MotorController m_frontLeftMotor;
  private MotorController m_frontRightMotor;
  private MotorController m_backLeftMotor;
  private MotorController m_backRightMotor;

  private Vector2d m_frontLeftVec;
  private Vector2d m_frontRightVec;
  private Vector2d m_backLeftVec;
  private Vector2d m_backRightVec;

  /**
   * Construct an Omni drive with the given motors and default motor angles.
   *
   * <p>The default motor angles make the wheels on each corner a 45 degree diagonal to that corner.
   *
   * <p>If a motor needs to be inverted, do so before passing it in.
   *
   * @param frontLeftMotor  The motor on the front left corner.
   * @param frontRightMotor  The motor on the front right corner.
   * @param backLeftMotor The motor on the back left corner.
   * @param backRightMotor  The motor on the back right corner.
   */
  public OmniDrive(MotorController frontLeftMotor, MotorController frontRightMotor, MotorController backLeftMotor,
                       MotorController backRightMotor) {
    this(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, kDefaultFrontLeftMotorAngle,
      kDefaultFrontRightMotorAngle, kDefaultBackLeftMotorAngle, kDefaultBackRightMotorAngle);
  }

  /**
   * Construct an Omni drive with the given motors.
   *
   * <p>Angles are measured in degrees clockwise from the positive X axis.
   *
   * @param frontLeftMotor  The motor on the front left corner.
   * @param frontRightMotor  The motor on the front right corner.
   * @param backLeftMotor The motor on the back left corner.
   * @param backRightMotor  The motor on the back right corner.
   * @param frontLeftMotorAngle  The angle of the front left wheel's forward direction of travel.
   * @param frontRightMotorAngle  The angle of the front right wheel's forward direction of travel.
   * @param backLeftMotorAngle The angle of the back left wheel's forward direction of travel.
   * @param backRightMotorAngle  The angle of the back right wheel's forward direction of travel.
   */
  public OmniDrive(MotorController frontLeftMotor,MotorController frontRightMotor, MotorController backLeftMotor,
                       MotorController backRightMotor, double frontLeftMotorAngle, double frontRightMotorAngle,
                       double backLeftMotorAngle, double backRightMotorAngle) {
    verify(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
    m_frontLeftMotor = frontLeftMotor;
    m_frontRightMotor = frontRightMotor;
    m_backLeftMotor = backLeftMotor;
    m_backRightMotor = backRightMotor;
    m_frontLeftVec = new Vector2d(Math.cos(frontLeftMotorAngle * (Math.PI / 180.0)),
                             Math.sin(frontLeftMotorAngle * (Math.PI / 180.0)));
    m_frontRightVec = new Vector2d(Math.cos(frontRightMotorAngle * (Math.PI / 180.0)),
                             Math.sin(frontRightMotorAngle * (Math.PI / 180.0)));
    m_backLeftVec = new Vector2d(Math.cos(backLeftMotorAngle * (Math.PI / 180.0)),
                              Math.sin(backLeftMotorAngle * (Math.PI / 180.0)));
    m_backRightVec = new Vector2d(Math.cos(backRightMotorAngle * (Math.PI / 180.0)),
                             Math.sin(backRightMotorAngle * (Math.PI / 180.0)));
    SendableRegistry.addChild(this, m_frontLeftMotor);
    SendableRegistry.addChild(this, m_frontRightMotor);
    SendableRegistry.addChild(this, m_backLeftMotor);
    SendableRegistry.addChild(this, m_backRightMotor);
    instances++;
    SendableRegistry.addLW(this, "OmniDrive", instances);
  }

  @Override
  public void close() {
    SendableRegistry.remove(this);
  }

  /**
   * Verifies that all motors are nonnull, throwing a NullPointerException if any of them are.
   * The exception's error message will specify all null motors, e.g. {@code
   * NullPointerException("leftMotor, rightMotor")}, to give as much information as possible to
   * the programmer.
   *
   * @throws NullPointerException if any of the given motors are null
   */
  @SuppressWarnings("PMD.AvoidThrowingNullPointerException")
  private void verify(MotorController frontLeftMotor, MotorController frontRightMotor, MotorController backLeftMotor,
                      MotorController backRightMotor) {
    if (frontLeftMotor != null && frontRightMotor != null && backLeftMotor != null && backRightMotor != null) {
      return;
    }
    StringJoiner joiner = new StringJoiner(", ");
    if (frontLeftMotor == null) {
      joiner.add("frontLeftMotor");
    }
    if (frontRightMotor == null) {
      joiner.add("frontRightMotor");
    }
    if (backLeftMotor == null) {
      joiner.add("backLeftMotor");
    }
    if (backRightMotor == null) {
      joiner.add("backRightMotor");
    }
    throw new NullPointerException(joiner.toString());
  }
  
  /**
   * Drive method for Omni platform.
   *
   * <p>Angles are measured clockwise from the positive X axis. The robot's speed is independent
   * from its angle or rotation rate.
   *
   * @param ySpeed    The robot's speed along the Y axis [-1.0..1.0]. Right is positive.
   * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *                  positive.
   */
  @SuppressWarnings("ParameterName")
  public void driveCartesian(double ySpeed, double xSpeed, double zRotation) {
    driveCartesian(ySpeed, xSpeed, zRotation, 0.0);
  }

  /**
   * Drive method for Omni platform.
   *
   * <p>Angles are measured clockwise from the positive X axis. The robot's speed is independent
   * from its angle or rotation rate.
   *
   * @param ySpeed    The robot's speed along the Y axis [-1.0..1.0]. Right is positive.
   * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *                  positive.
   * @param gyroAngle The current angle reading from the gyro in degrees around the Z axis. Use
   *                  this to implement field-oriented controls.
   */
  @SuppressWarnings("ParameterName")
  public void driveCartesian(double ySpeed, double xSpeed, double zRotation,
                             double gyroAngle) {

    ySpeed = MathUtil.clamp(ySpeed, -1.0, 1.0);
    ySpeed = MathUtil.applyDeadband(ySpeed, m_deadband);

    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    xSpeed = MathUtil.applyDeadband(xSpeed, m_deadband);

    // Compensate for gyro angle.
    Vector2d input = new Vector2d(ySpeed, xSpeed);
    input.rotate(-gyroAngle);

    double[] wheelSpeeds = new double[4];
    wheelSpeeds[0] = input.scalarProject(m_frontLeftVec) + zRotation;
    wheelSpeeds[1] = input.scalarProject(m_frontRightVec) + zRotation;
    wheelSpeeds[2] = input.scalarProject(m_backLeftVec) + zRotation;
    wheelSpeeds[3] = input.scalarProject(m_backRightVec) + zRotation;

    normalize(wheelSpeeds);

    m_frontLeftMotor.set(wheelSpeeds[0] * m_maxOutput);
    m_frontRightMotor.set(wheelSpeeds[1] * m_maxOutput);
    m_backLeftMotor.set(wheelSpeeds[2] * m_maxOutput);
    m_backRightMotor.set(wheelSpeeds[3] * m_maxOutput);

    feed();
  }

  /**
   * Drive method for Omni platform.
   *
   * <p>Angles are measured counter-clockwise from straight ahead. The speed at which the robot
   * drives (translation) is independent from its angle or rotation rate.
   *
   * @param magnitude The robot's speed at a given angle [-1.0..1.0]. Forward is positive.
   * @param angle     The angle around the Z axis at which the robot drives in degrees [-180..180].
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
   *                  positive.
   */
  @SuppressWarnings("ParameterName")
  public void drivePolar(double magnitude, double angle, double zRotation) {
    driveCartesian(magnitude * Math.sin(angle * (Math.PI / 180.0)),
                   magnitude * Math.cos(angle * (Math.PI / 180.0)), zRotation, 0.0);
  }

  @Override
  public void stopMotor() {
    m_frontLeftMotor.stopMotor();
    m_frontRightMotor.stopMotor();
    m_backLeftMotor.stopMotor();
    m_backRightMotor.stopMotor();
    feed();
  }

  @Override
  public String getDescription() {
    return "OmniDrive";
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("OmniDrive");
    builder.setActuator(true);
    builder.setSafeState(this::stopMotor);
    builder.addDoubleProperty("Front Left Motor Speed", m_frontLeftMotor::get, m_frontLeftMotor::set);
    builder.addDoubleProperty("Front Right Motor Speed", m_frontRightMotor::get, m_frontRightMotor::set);
    builder.addDoubleProperty("Back Left Motor Speed", m_backLeftMotor::get, m_backLeftMotor::set);
    builder.addDoubleProperty("Back Right Motor Speed", m_backRightMotor::get, m_backRightMotor::set);
  }
}
