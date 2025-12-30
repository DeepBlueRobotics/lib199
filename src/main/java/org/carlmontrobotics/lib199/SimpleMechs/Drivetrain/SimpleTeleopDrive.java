package org.carlmontrobotics.lib199.SimpleMechs.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SimpleTeleopDrive extends Command {

  private static double robotPeriod = 0.2;
  private final SimpleDrivetrain drivetrain;
  private DoubleSupplier fwd;
  private DoubleSupplier str;
  private DoubleSupplier rcw;
  private BooleanSupplier slow;
  private double currentForwardVel = 0;
  private double currentStrafeVel = 0;
  private double prevTimestamp;
  GenericHID manipulatorController;
  BooleanSupplier babyModeSupplier;


  //Extra
  private final double maxForward;
  private final double maxStrafe;
  private final double maxRCW;
  private final double kSlowDriveSpeed = 0.4;
  private final double kNormalDriveSpeed = 1;
  private final double kSlowDriveRotation = 0.25;
  private final double kNormalDriveRotation = 0.5;
  private final double kBabyDriveSpeed = 0.3;
  private final double kBabyDriveRotation = 0.2;
  private final double JOY_THRESH = 0.13;


  /**
   * Creates a new TeleopDrive.
   */
  public SimpleTeleopDrive(SimpleDrivetrain drivetrain, DoubleSupplier fwd, DoubleSupplier str, DoubleSupplier rcw,
      BooleanSupplier slow, GenericHID manipulatorController, BooleanSupplier babyModeSupplier, double maxSpeed, double swerveRadius ) {
    addRequirements(this.drivetrain = drivetrain);
    this.fwd = fwd;
    this.str = str;
    this.rcw = rcw;
    this.slow = slow;
    this.manipulatorController = manipulatorController;
    this.babyModeSupplier = babyModeSupplier;
    maxForward = maxSpeed;
    maxStrafe = maxSpeed;
    maxRCW = maxSpeed/swerveRadius;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard.putNumber("slow turn const", kSlowDriveRotation);
    // SmartDashboard.putNumber("slow speed const", kSlowDriveSpeed);
    // SmartDashboard.putNumber("normal turn const", kNormalDriveRotation);
    // SmartDashboard.putNumber("normal speed const", kNormalDriveSpeed);
    prevTimestamp = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = Timer.getFPGATimestamp();
    robotPeriod = currentTime - prevTimestamp;
    if (!hasDriverInput()) {
      drivetrain.drive(0,0,0);
    }
    else {
    double[] speeds = getRequestedSpeeds();
    // SmartDashboard.putNumber("Elapsed time", currentTime - prevTimestamp);
    prevTimestamp = currentTime;
    // kSlowDriveRotation = SmartDashboard.getNumber("slow turn const", kSlowDriveRotation);
    // kSlowDriveSpeed = SmartDashboard.getNumber("slow speed const", kSlowDriveSpeed);
    // kNormalDriveRotation = SmartDashboard.getNumber("normal turn const", kNormalDriveRotation);
    // kNormalDriveSpeed = SmartDashboard.getNumber("normal speed const", kNormalDriveSpeed);

    // SmartDashboard.putNumber("fwd", speeds[0]);
    // SmartDashboard.putNumber("strafe", speeds[1]);
    // SmartDashboard.putNumber("turn", speeds[2]);
      drivetrain.drive(speeds[0], speeds[1], speeds[2]);
    }
  }

  public double[] getRequestedSpeeds() {
    // Sets all values less than or equal to a very small value (determined by the
    // idle joystick state) to zero.
    // Used to make sure that the robot does not try to change its angle unless it
    // is moving,
    double forward = fwd.getAsDouble();
    double strafe = str.getAsDouble();
    double rotateClockwise = rcw.getAsDouble();
    // SmartDashboard.putNumber("fwdIN", forward);
    // SmartDashboard.putNumber("strafeIN", strafe);
    // SmartDashboard.putNumber("turnIN", rotateClockwise);
    // System.out.println("fwd str rcw: "+forward+", "+strafe+", "+rotateClockwise);
    boolean slow2 = slow.getAsBoolean();
    forward *= maxForward;
    strafe *= maxStrafe;
    rotateClockwise *= maxRCW;

    // System.out.println("teleopDrive ExtraSpeedMult%: "+drivetrain.extraSpeedMult);
    double driveMultiplier = (slow.getAsBoolean() ? kSlowDriveSpeed : kNormalDriveSpeed);
    double rotationMultiplier = drivetrain.extraSpeedMult + (slow.getAsBoolean() ? kSlowDriveRotation : kNormalDriveRotation);
    // double driveMultiplier = (slow.getAsBoolean() ? kSlowDriveSpeed : kNormalDriveSpeed);
    // double rotationMultiplier = (slow.getAsBoolean() ? kSlowDriveRotation : kNormalDriveRotation);
    
    if(babyModeSupplier.getAsBoolean()){
      driveMultiplier = kBabyDriveSpeed; 
      rotationMultiplier = kBabyDriveRotation;
    }
    // double driveMultiplier = kNormalDriveSpeed;
    // double rotationMultiplier = kNormalDriveRotation;

    forward *= driveMultiplier;
    strafe *= driveMultiplier;
    rotateClockwise *= rotationMultiplier;

    // Limit acceleration of the robot
    // double accelerationX = (forward - currentForwardVel) / robotPeriod;
    // double accelerationY = (strafe - currentStrafeVel) / robotPeriod;
    // double translationalAcceleration = Math.hypot(accelerationX, accelerationY);
    // SmartDashboard.putNumber("Translational Acceleration", translationalAcceleration);
    // if (translationalAcceleration > autoMaxAccelMps2 && false) {//DOES NOT RUN!!
    //   Translation2d limitedAccelerationVector = new Translation2d(autoMaxAccelMps2,
    //       Rotation2d.fromRadians(Math.atan2(accelerationY, accelerationX)));
    //   Translation2d limitedVelocityVector = limitedAccelerationVector.times(robotPeriod);
    //   currentForwardVel += limitedVelocityVector.getX();
    //   currentStrafeVel += limitedVelocityVector.getY();
    // } else {
    currentForwardVel = forward;
    currentStrafeVel = strafe;
    // }
    // SmartDashboard.putNumber("current velocity", Math.hypot(currentForwardVel, currentStrafeVel));

    return new double[] { currentForwardVel, currentStrafeVel, -rotateClockwise };
  }

  public boolean hasDriverInput() {
    return MathUtil.applyDeadband(fwd.getAsDouble(), JOY_THRESH)!=0
        || MathUtil.applyDeadband(str.getAsDouble(), JOY_THRESH)!=0
        || MathUtil.applyDeadband(rcw.getAsDouble(), JOY_THRESH)!=0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}