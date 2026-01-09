// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Important info about SimpleArm!
 * Encoders are set to read in degrees from the perspective of the rotating object, with 0 being horizontal in the more often direction the use would use
 */

package org.carlmontrobotics.lib199.SimpleMechs.Arm;


import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.SimpleMechs.Elastic;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;

public class SimpleArm extends SubsystemBase {
  /** Creates a new SimpleArm. */
  private final ArmConfig armConfig;
  private SparkBase armMasterMotor;
  private RelativeEncoder armFeedbackEncoder;
  private AbsoluteEncoder armResetEncoder;
  private RelativeEncoder armBackupEncoder;
  private SparkBase[] armFollowMotors;
  private double encoderDif = 0;
  private final double difDanger = 1;

  private double armGoal; // in degrees
  private double latestManualInput; // in percentage of power (-1, 1)

  private ArmControlState currentState = ArmControlState.AUTO;
  private IdleMode armIdleMode = IdleMode.kBrake;

  private final double defaultRelativeEncoderResetValue = -90;

  private double currentStamp;
  private double pastStamp;
  private double currentPeriod = 0.02;

  private boolean encoderFaultReported = false;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle m_angle = Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

  private SysIdRoutine.Config defaultSysIdConfig = new SysIdRoutine.Config(
    Volts.of(1).per(Seconds),//ramp rate, volts/sec
    Volts.of(1), //starting voltage, volts
    Seconds.of(5)//maximum sysID test time
  );


  private final SysIdRoutine sysIdRoutine;

  /**
   * Control State for the arm
   * {@link #AUTO} For automatically controling the arm to go to a certain goal
   * {@link #MANUAL} For manually controlling the arm by providing percentage of voltage
   * {@link #BRAKE} For holding the arm in a rigidish state, not moving
   * {@link #COAST} For setting the arm to not resist movement, allowing accessibility when the robot is disabled.
   */
  public enum ArmControlState {
    AUTO,
    MANUAL,
    BRAKE,
    COAST
  }

  /**
   * Creates an object of {@link SimpleArm}, can be extended off this class to create a more complicated arm without requiring to fully remake a subsystem.
   * @param armConfig
   */
  public SimpleArm(ArmConfig armConfig) {
    this.armConfig = armConfig;
    armResetEncoder = armConfig.armMainAbsoluteEncoder;
    configMotors();
    resetFeedbackEncoder();
    sysIdRoutine = new SysIdRoutine(
      defaultSysIdConfig,
      new SysIdRoutine.Mechanism(
        voltage -> {
          manualArm(voltage);
        },
        log -> {
          log.motor("Spinner-Master")
            .voltage(
              m_appliedVoltage
            .mut_replace(armMasterMotor.getBusVoltage() * armMasterMotor.getAppliedOutput(), Volts))
            .angularPosition(m_angle.mut_replace(Units.degreesToRadians(armFeedbackEncoder.getPosition()), Radians))
            .angularVelocity(m_velocity.mut_replace(Units.degreesToRadians(armFeedbackEncoder.getVelocity()), RadiansPerSecond));
        },
        this)
      );
    
    if (isSysIdTesting()) {
      sysIdSetup();
    }
    SmartDashboard.putData(this);
  }

  /**
   * Sets up commands in SmartDashboard/Elastic to be ran to record data for mechanism modeling
   */
  private void sysIdSetup() {
    Elastic.Notification notification = new Elastic.Notification(Elastic.NotificationLevel.INFO, "SysId Testing On", "Please add Arm Quasistatic and Dynamics and begin testing.").withDisplaySeconds(5);
    Elastic.sendNotification(notification);
    Elastic.selectTab("SysId");
    SmartDashboard.putData("Arm Quasistatic Backward", sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("Arm Quasistatic Forward", sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("Arm Dynamic Forward", sysIdDynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("Arm Dynamic Backward", sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /** Override this method and return true if you want to do sysid testing, return false if you don't
   * @return false to do sysid testing
   */
  protected boolean isSysIdTesting() {
    return false;
  }

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  } 

  /**
   * @return position in degrees from horizontal
   */
  public double getArmPosition() {
    return armFeedbackEncoder.getPosition();
  }


  /**
   * Sets {@link #armGoal}, as long as the value is within the provided limits in the {@link ArmConfig}.
   * @param goal in degrees.
   */
  public void setGoal(double goal){
    if (goal <= armConfig.topLimit && goal >= armConfig.bottomLimit) {armGoal = goal;}
  }

  /**
   * Gets the current value of {@link #armGoal}.
   * @return {@link #armGoal} in degrees.
   */
  public double getGoal() {
    return armGoal;
  }

  /**
   * Checks if {@link SimpleArm} is at {@link #armGoal} within the given tolerance in the {@link ArmConfig}.
   * @return true if it is at the {@link #armGoal}.
   */
  public boolean atGoal() {
    return Math.abs(armGoal - armFeedbackEncoder.getPosition()) <= armConfig.armPIDTolerance;
  }

  /**
   * Sets {@link #latestManualInput}.
   * @param input -1.0 - 1.0 range for inputing voltage percentage into the arm.
   */
  public void setManualInput(double input) {
    latestManualInput = input;
  }

  /**
   * Fetches the user input being applied on the arm if the {@link #currentState} is MANUAL.
   * @return {@link #latestManualInput}
   */
  public double getLatestManualInput() {
    return latestManualInput;
  }

  /**
   * @return active {@link ArmControlState}.
   */
  protected ArmControlState getCurrentState() {
    return currentState;
  } 

  /**
   * Toggles {@link #currentState} between AUTO and MANUAL
   * Does not do anything if arm is idling
   */
  public void toggleControlMode() {
    if (currentState == ArmControlState.AUTO) {
      currentState = ArmControlState.MANUAL;
    }
    else if (currentState == ArmControlState.MANUAL) {
      currentState = ArmControlState.AUTO;
    }
  }

  /**
   * Sets {@link #currentState} to be a certain {@link ArmControlState}
   * @param controlState desired control state
   */
  public void setArmControlState(ArmControlState controlState) {
    switch (controlState) {
      case AUTO -> setAutoOn();
      case MANUAL -> setManualOn();
      case BRAKE -> setBrakeOn();
      case COAST -> setCoastOn();
      default -> DriverStation.reportWarning("Such control mode has not been implemented yet", true);
    }
  }

  /**
   * Sets {@link #currentState} to be AUTO making the arm approach the {@link #armGoal}
   */
  public void setAutoOn() {
    if (armConfig.armPID != null || armConfig.armFeedForward != null) {
      currentState = ArmControlState.AUTO;
    }
    else {
      DriverStation.reportWarning("Any sort of autonomous control mode is disabled due to no PID or FeedForward", true);
    }
  }

  /**
   * Sets {@link #currentState} to be MANUAL allowing user to move the arm manually
   */
  public void setManualOn() {
    currentState = ArmControlState.MANUAL;
  }

  /**
   * Sets {@link #currentState} to be BRAKE and brakes the motors
   */
  public void setBrakeOn() {
    currentState = ArmControlState.BRAKE;
    if (armIdleMode == IdleMode.kCoast) {
      armIdleMode = IdleMode.kBrake;
      brakeMotors();

    }
  }

  /**
   * Sets {@link #currentState} to be COAST and coasts the motors
   */
  public void setCoastOn() {
    currentState = ArmControlState.COAST;
    if (armIdleMode == IdleMode.kBrake) {
      armIdleMode = IdleMode.kCoast;
      coastMotors();
    }
  }

  /**
   * Resets feedback encoder using the {@link #armResetEncoder}, if such does not exist will use a {@link #defaultRelativeEncoderResetValue}
   */
  public void resetFeedbackEncoder() {
    if (armConfig.armMainAbsoluteEncoder != null) {
      armFeedbackEncoder.setPosition((armResetEncoder.getPosition() + armConfig.armAbsoluteZeroOffset)/armConfig.armAbsoluteGearReduction*armConfig.armRelativeGearReduction);
      if (armBackupEncoder == null) {return;}
      armBackupEncoder.setPosition((armResetEncoder.getPosition() + armConfig.armAbsoluteZeroOffset)/armConfig.armAbsoluteGearReduction*armConfig.armRelativeGearReduction);
    }
    else {
      armFeedbackEncoder.setPosition(defaultRelativeEncoderResetValue);
      if (armBackupEncoder == null) {return;}
      armBackupEncoder.setPosition(defaultRelativeEncoderResetValue);
    }
  }

  /**
   * Called periodically
   * Checks difference between active encoder and backup encoder to see if there seems to be a mishap, such as encoder not measuring properly or just turning off.
   */
  private void checkFeedBackEncoder() {
    if (armBackupEncoder == null) {return;}
    encoderDif = armFeedbackEncoder.getPosition() - armBackupEncoder.getPosition();
    if (Math.abs(encoderDif) >= difDanger && !encoderFaultReported) {
      encoderFaultReported = true;
      DriverStation.reportWarning("Arm encoder seems to be off!", true);
    }
  }
    
  /**
   * Called in periodic if {@link #currentState} is auto.
   * Will calculate the desired voltage to achieve the {@link #armGoal}
   */
  protected void autoArm() {
    double pidOutput;
    double feedforwardOutput;
    double curPosition = armFeedbackEncoder.getPosition();
    if (armConfig.armPID != null) {
      pidOutput = armConfig.armPID.calculate(curPosition, armGoal);
    }
    else {
      pidOutput = 0;
    }

    if (armConfig.armFeedForward != null) {
      feedforwardOutput = armConfig.armFeedForward.calculate(Units.degreesToRadians(curPosition), 0);
    }
    else {
      feedforwardOutput = 0;
    }
    if (curPosition < armConfig.topLimit) {
      if (curPosition > armConfig.bottomLimit) {
        armMasterMotor.setVoltage(MathUtil.clamp(pidOutput + feedforwardOutput, -armConfig.armMaxVolts, armConfig.armMaxVolts));
      }
      else {
        armMasterMotor.setVoltage(MathUtil.clamp(pidOutput + feedforwardOutput, 0, armConfig.armMaxVolts));
      }
    }
    else {
      armMasterMotor.setVoltage(MathUtil.clamp(pidOutput + feedforwardOutput, -armConfig.armMaxVolts, 0));
    }
  }

  /**
   * Called in periodic if {@link #currentState} is manual.
   * Will apply a percentage of voltage to the arm based of the users input: {@link #latestManualInput}, while soft locking it to avoid the arm leaving the intended bounds
   */
  protected void manualArm() {
    double curPosition = armFeedbackEncoder.getPosition();
    if (curPosition < armConfig.topLimit) {
      if (curPosition > armConfig.bottomLimit) {
        armMasterMotor.set(MathUtil.clamp(latestManualInput, -armConfig.armMaxVolts/12, armConfig.armMaxVolts/12));
      }
      else {
        armMasterMotor.set(MathUtil.clamp(latestManualInput, 0, armConfig.armMaxVolts/12));
      }
    }
    else {
      armMasterMotor.set(MathUtil.clamp(latestManualInput, -armConfig.armMaxVolts/12, 0));
    }
  }

  /**
   * Mainly used by SysId to characterize the spinner model
   * @param volts Voltage unit input
   */
  protected void manualArm(Voltage volts) {
    double curPosition = armFeedbackEncoder.getPosition();
    double voltage = volts.in(Volts);
    if (curPosition < armConfig.topLimit) {
      if (curPosition > armConfig.bottomLimit) {
        armMasterMotor.setVoltage(MathUtil.clamp(voltage, -armConfig.armMaxVolts, armConfig.armMaxVolts));
      }
      else {
        armMasterMotor.setVoltage(MathUtil.clamp(voltage, 0, armConfig.armMaxVolts));
      }
    }
    else {
      armMasterMotor.setVoltage(MathUtil.clamp(voltage, -armConfig.armMaxVolts, 0));
    }
  }

  private void configMotors() {
    configFollowMotors();
    configMasterMotor();
  }

  /**
   * Configures follow motors to follow the master, and also to brake, and have the correct inverse for encoder readings, and to follow the master with the correct inverse.
   */
  protected void configFollowMotors() {
    if (armConfig.armFollowId != null) {
      armFollowMotors = new SparkBase[armConfig.armFollowId.length];

      SparkFlexConfig flexFollowConfig = new SparkFlexConfig();
      flexFollowConfig.idleMode(IdleMode.kBrake);
      SparkMaxConfig maxFollowConfig = new SparkMaxConfig();
      maxFollowConfig.idleMode(IdleMode.kBrake);

      for (int i = 0; i < armConfig.armFollowId.length; i++) {
        if (armConfig.armFollowId[i] == armConfig.armMotorOfBackupRelativeEncoderId) {
          if (armConfig.armFollowMotorType[i] == MotorConfig.NEO_VORTEX) {
              SparkFlexConfig flexConfig = new SparkFlexConfig();
              SparkFlex dummyFlex = MotorControllerFactory.createSparkFlex(armConfig.armFollowId[i]);
              flexConfig.encoder.positionConversionFactor(armConfig.armRelativeGearReduction * 360) // Degrees
                                .velocityConversionFactor(armConfig.armRelativeGearReduction/60 * 360); //degrees per second
              dummyFlex.configure(flexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
              armBackupEncoder = dummyFlex.getEncoder();
          }
          else {
              SparkMaxConfig maxConfig = new SparkMaxConfig();
              SparkMax dummyMax = MotorControllerFactory.createSparkMax(armConfig.armFollowId[i], armConfig.armFollowMotorType[i]);
              maxConfig.encoder.positionConversionFactor(armConfig.armRelativeGearReduction * 360) // Degrees
                              .velocityConversionFactor(armConfig.armRelativeGearReduction/60 *360); //degrees per second
              dummyMax.configure(maxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
              armBackupEncoder = dummyMax.getEncoder();
          }
        }
        if (armConfig.armFollowMotorType[i] == MotorConfig.NEO_VORTEX) {
          flexFollowConfig.follow(armConfig.armMasterMotorId, !(armConfig.armMasterInversed && armConfig.armFollowInversed[i]));
          SparkFlex followFlex = MotorControllerFactory.createSparkFlex(armConfig.armFollowId[i]);
          followFlex.configure(flexFollowConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
          armFollowMotors[i] = followFlex;
        }
        else {
          maxFollowConfig.follow(armConfig.armMasterMotorId, !(armConfig.armMasterInversed && armConfig.armFollowInversed[i]));
          SparkMax followMax = MotorControllerFactory.createSparkMax(armConfig.armFollowId[i], armConfig.armFollowMotorType[i]);
          followMax.configure(maxFollowConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
          armFollowMotors[i] = followMax;
        }
      }
    }
  }

  /**
   * Configures master motor with brake and correct inverse config
   */
  protected void configMasterMotor() {
    if (armConfig.armMasterMotorType == MotorConfig.NEO_VORTEX) {
      SparkFlexConfig flexConfig = new SparkFlexConfig();
      this.armMasterMotor = MotorControllerFactory.createSparkFlex(armConfig.armMasterMotorId);
      flexConfig.inverted(armConfig.armMasterInversed)
                .idleMode(IdleMode.kBrake)
                .encoder.positionConversionFactor(armConfig.armRelativeGearReduction * 360) // Degrees
                        .velocityConversionFactor(armConfig.armRelativeGearReduction/60 * 360); //degrees per second
      armMasterMotor.configure(flexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      armFeedbackEncoder = armMasterMotor.getEncoder();
  }
  else {
      SparkMaxConfig maxConfig = new SparkMaxConfig();
      armMasterMotor = MotorControllerFactory.createSparkMax(armConfig.armMasterMotorId, armConfig.armMasterMotorType);
      maxConfig.inverted(armConfig.armMasterInversed)
                .idleMode(IdleMode.kBrake)
                .encoder.positionConversionFactor(armConfig.armRelativeGearReduction * 360) // Degrees
                        .velocityConversionFactor(armConfig.armRelativeGearReduction/60* 360); //degrees per second
      armMasterMotor.configure(maxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      armFeedbackEncoder = armMasterMotor.getEncoder();
  }
  }

  /**
   * Set all motors idleModes to brake
   */
  private void brakeMotors() {
    SparkFlexConfig flexBrakeConfig = new SparkFlexConfig();
    SparkMaxConfig maxBrakeConfig = new SparkMaxConfig();

    flexBrakeConfig.idleMode(IdleMode.kBrake);
    maxBrakeConfig.idleMode(IdleMode.kBrake);

    if (armConfig.armFollowId != null) {
      for (int i = 0; i < armFollowMotors.length; i++) {
        if (armConfig.armFollowMotorType[i] == MotorConfig.NEO_VORTEX) {
          armFollowMotors[i].configure(flexBrakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }
        else {
          armFollowMotors[i].configure(maxBrakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }
      }
    }
    if (armConfig.armMasterMotorType == MotorConfig.NEO_VORTEX) {
      armMasterMotor.configure(flexBrakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    else {
      armMasterMotor.configure(maxBrakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  /**
   * Set all motors idleModes to coast
   */
  private void coastMotors() {
    SparkFlexConfig flexCoastConfig = new SparkFlexConfig();
    SparkMaxConfig maxCoastConfig = new SparkMaxConfig();

    flexCoastConfig.idleMode(IdleMode.kCoast);
    maxCoastConfig.idleMode(IdleMode.kCoast);

    if (armConfig.armFollowId != null) {
      for (int i = 0; i < armFollowMotors.length; i++) {
        if (armConfig.armFollowMotorType[i] == MotorConfig.NEO_VORTEX) {
          armFollowMotors[i].configure(flexCoastConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }
        else {
          armFollowMotors[i].configure(maxCoastConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }
      }
    }
    if (armConfig.armMasterMotorType == MotorConfig.NEO_VORTEX) {
      armMasterMotor.configure(flexCoastConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    else {
      armMasterMotor.configure(maxCoastConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  public void stopArm() {
    armMasterMotor.set(0);
  }

  /**
   * Use this to do any fun stuff you want to do, for armSubsystemPeriodic
   */
  protected void userPeriodic() {

  }

  protected void postSmartDashboardValues() {
    SmartDashboard.putNumber("armVelocity (Degrees/s)", armFeedbackEncoder.getVelocity());
    SmartDashboard.putNumber("armPosition (Degrees)", armFeedbackEncoder.getPosition()); //From horizontal being 0 and up is positive
    switch (currentState) {
      case AUTO -> SmartDashboard.putString("armControlState", "Autonomous");
      case MANUAL -> SmartDashboard.putString("armControlState", "Manual");
      case BRAKE -> SmartDashboard.putString("armControlState", "Idling: Brake");
      case COAST -> SmartDashboard.putString("armControlState", "Idling: Coast");
    }
    SmartDashboard.putNumber("armGoal (Degrees from horizontal)", armGoal);
    SmartDashboard.putNumber("Current period of Arm (ms)", currentPeriod*1000);

  }

  /**
   * Checks Subsystem periodic for running under expected time limits, reports inconsitencies to user, also is usable by feedforward
   */
  private void checkPeriodic() {
    currentStamp = Timer.getFPGATimestamp();
    currentPeriod = currentStamp - pastStamp;
    pastStamp = currentStamp;
    if (currentPeriod > 0.02) {
      DriverStation.reportWarning("Arm Periodic is slow: " + currentPeriod *1000 + " ms. Check for any lag reasons", true);
    }
  }

  /**
   * @return current calculated period in seconds
   */
  protected double getCurrentPeriod() {
    return currentPeriod;
  }

  /*Overide these methods to customize what the mechanism does in different states */
  protected void handleAuto()   { autoArm();}
  protected void handleManual() { manualArm();}
  protected void handleBrake()  { stopArm();}
  protected void handleCoast()  { stopArm();}


  /** 
   * DO NOT OVERRIDE THIS PERIODIC UNLESS YOU KNOW WHAT YOU ARE DOING!
   * Use {@link #userPeriodic()} instead.
   */
  @Override
  public void periodic() {
    switch (currentState) {
      case AUTO -> handleAuto();
      case MANUAL -> handleManual();
      case BRAKE -> handleBrake();
      case COAST -> handleCoast();
    }
    checkFeedBackEncoder();
    userPeriodic();
    postSmartDashboardValues();
    checkPeriodic();
  }

}
