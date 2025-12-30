// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.lib199.SimpleMechs.Spinner;


import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.SimpleMechs.Elastic;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularAcceleration;
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
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;

public class SimpleSpinner extends SubsystemBase {
  private final SpinnerConfig spinnerConfig;
  private SparkBase masterMotor;
  private SparkBase followMotor;
  private RelativeEncoder feedbackEncoder;
  private RelativeEncoder backupEncoder;
  private double spinnerGoal;
  private double latestManualInput; // in percentage of power (-1, 1)

  private SpinnerControlState currentState = SpinnerControlState.AUTO;
  private IdleMode spinnerIdleMode = IdleMode.kBrake;

  private double currentDif = 0;
  private final double difDanger = 0.2;

  private double currentStamp = 0;
  private double pastStamp = 0;
  private double currentPeriod = 0.02;

  private double currentAcceleration = 0.0;
  private double previousVelocity = 0;

  private boolean encoderFaultReported = false;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle m_angle = Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);
  // Mutable holder for unit-safe linear accleration values, persisted to avoid reallocation
  private final MutAngularAcceleration m_accleration = RadiansPerSecondPerSecond.mutable(0);

  private SysIdRoutine.Config defaultSysIdConfig = new SysIdRoutine.Config(
    Volts.of(1).per(Seconds),//ramp rate, volts/sec
    Volts.of(1), //starting voltage, volts
    Seconds.of(12)//maximum sysID test time
  );


  private final SysIdRoutine sysIdRoutine;


    /**
   * Different control states for the spinner
   * {@link #AUTO} runs the spinner autonomously to go to a certain speed
   * {@link #MANUAL} allows the spinner to be manually run by the user with speed limits
   * {@link #BRAKE} idles the spinner in brake which hold the spinner a certain position
   * {@link #COAST} idles the spinner in coast allowing for easy movement
   */
  public enum SpinnerControlState {
    AUTO,
    MANUAL,
    BRAKE,
    COAST
  } 

  public SimpleSpinner(SpinnerConfig spinnerConfig) {
    this.spinnerConfig = spinnerConfig;
    configureMotors();
    resetFeedbackEncoder();
    sysIdRoutine = new SysIdRoutine(
      defaultSysIdConfig,
      new SysIdRoutine.Mechanism(
        voltage -> {
          manualSpinner(voltage);
        },
        log -> {
          log.motor("Spinner-Master")
            .voltage(
              m_appliedVoltage
            .mut_replace(masterMotor.getBusVoltage() * masterMotor.getAppliedOutput(), Volts))
            .angularPosition(m_angle.mut_replace(feedbackEncoder.getPosition()*Math.PI*2, Radians))
            .angularVelocity(m_velocity.mut_replace(getSpinnerVelocity()*Math.PI*2, RadiansPerSecond))
            .angularAcceleration(m_accleration.mut_replace(getSpinnerAccleration()*Math.PI*2, RadiansPerSecondPerSecond));
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
    Elastic.Notification notification = new Elastic.Notification(Elastic.NotificationLevel.INFO, "SysId Testing On", "Please add Spinner Quasistatic and Dynamics and begin testing.").withDisplaySeconds(5);
    Elastic.sendNotification(notification);
    Elastic.selectTab("SysId");
    SmartDashboard.putData("Spinner Quasistatic Backward", sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("Spinner Quasistatic Forward", sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("Spinner Dynamic Forward", sysIdDynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("Spinner Dynamic Backward", sysIdDynamic(SysIdRoutine.Direction.kReverse));
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
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  } 

  /**
   * Sets {@link #spinnerGoal}
   * @param goal in rotations per second
   */
  public void setGoal(double goal){
    spinnerGoal = goal;
  }

  /**
   * @return current {@link #spinnerGoal} in rotations per second
   */
  public double getGoal() {
    return spinnerGoal;
  }

  /**
   * Checks if at {@link #spinnerGoal}
   * @return true if at {@link #spinnerGoal}
   */
  public boolean atGoal() {
    return Math.abs(spinnerGoal - feedbackEncoder.getVelocity()) <= spinnerConfig.spinnerPIDTolerance;
  }

  /**
   * Updates {@link #latestManualInput} to manually control spinner
   * @param input voltage percentage from -1.0 - 1.0
   */
  public void setManualInput(double input) {
    latestManualInput = input;
  }

  /**
   * @return {@link #latestManualInput} a value from -1.0 - 1.0
   */
  public double getLatestManualInput() {
    return latestManualInput;
  }

  /**
  * @return {@link SpinnerConfig} of the object
  */
  public SpinnerConfig getConfig() {
    return spinnerConfig;
  }

  /**
   * Sets {@link #currentState} to the inputed state
   * @param controlState An {@link SpinnerControlState}
   */
  public void setSpinnerControlState(SpinnerControlState controlState) {
    switch (controlState) {
      case AUTO -> setAutoOn();
      case MANUAL -> setManualOn();
      case BRAKE -> setBrakeOn();
      case COAST -> setCoastOn();
      default -> DriverStation.reportWarning("Such control mode has not been implemented yet", true);
    }
  }


  /**
   * @return current spinner velocity in rotations per second
   */
  public double getSpinnerVelocity() {
    return feedbackEncoder.getVelocity();
  }

  /**
   * @return current spinner acceleration in rotations per second per second
   */
  public double getSpinnerAccleration() {
    return currentAcceleration;
  }

    /**
   * Sets {@link #currentState} to the inputed state
   * @param controlState An {@link SpinnerControlState}
   */
  public void setElevatorControlState(SpinnerControlState controlState) {
    switch (controlState) {
      case AUTO -> setAutoOn();
      case MANUAL -> setManualOn();
      case BRAKE -> setBrakeOn();
      case COAST -> setCoastOn();
      default -> DriverStation.reportWarning("Such control mode has not been implemented yet", true);
    }
  }

  /**
   * Sets {@link #currentState} to be in AUTO mode
   */
  public void setAutoOn() {
    if (spinnerConfig.spinnerPIDExists || spinnerConfig.spinnerFeedForwardExists) {
      currentState = SpinnerControlState.AUTO;
    }
    else {
      DriverStation.reportWarning("Any sort of autonomous control mode is disabled due to no PID or FeedForward", true);
    }
  }

  /**
   * Sets {@link #currentState} to be in MANUAL mode
   */
  public void setManualOn() {
    currentState = SpinnerControlState.MANUAL;
  }

  /**
   * Sets {@link #currentState} to be in BRAKE mode, also changes the idle mode of the motors to brake.
   */
  public void setBrakeOn() {
    currentState = SpinnerControlState.BRAKE;
    if (spinnerIdleMode == IdleMode.kCoast) {
      spinnerIdleMode = IdleMode.kBrake;
      brakeMotors();

    }
  }

  /**
   * Sets {@link #currentState} to be in COAST mode, also changes the idle mode of the motors to coast.
   */
  public void setCoastOn() {
    currentState = SpinnerControlState.COAST;
    if (spinnerIdleMode == IdleMode.kBrake) {
      spinnerIdleMode = IdleMode.kCoast;
      coastMotors();
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
    if (spinnerConfig.spinnerFollowExists) {
      if (spinnerConfig.spinnerFollowMotorType == MotorConfig.NEO_VORTEX) {
        followMotor.configure(flexBrakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      }
      else {
        followMotor.configure(maxBrakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      }
    }
    if (spinnerConfig.spinnerMasterMotorType == MotorConfig.NEO_VORTEX) {
      masterMotor.configure(flexBrakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    else {
      masterMotor.configure(maxBrakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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
    if (spinnerConfig.spinnerFollowExists) {
      if (spinnerConfig.spinnerFollowMotorType == MotorConfig.NEO_VORTEX) {
        followMotor.configure(flexCoastConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      }
      else {
        followMotor.configure(maxCoastConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      }
    }
    if (spinnerConfig.spinnerMasterMotorType == MotorConfig.NEO_VORTEX) {
      masterMotor.configure(flexCoastConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    else {
      masterMotor.configure(maxCoastConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
  }


  protected void configureMotors() {
    if (spinnerConfig.spinnerMasterMotorType == MotorConfig.NEO_VORTEX) {
      SparkFlexConfig masterFlexConfig = new SparkFlexConfig();
      masterFlexConfig.idleMode(IdleMode.kBrake)
                      .inverted(spinnerConfig.spinnerMasterInverted)
                      .encoder.positionConversionFactor(spinnerConfig.gearReduction)
                      .velocityConversionFactor(spinnerConfig.gearReduction/60);
      masterMotor = MotorControllerFactory.createSparkFlex(spinnerConfig.spinnerMasterId);
      masterMotor.configure(masterFlexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    else {
      SparkMaxConfig masterMaxConfig = new SparkMaxConfig();
      masterMaxConfig.idleMode(IdleMode.kBrake)
                     .inverted(spinnerConfig.spinnerMasterInverted)
                     .encoder.positionConversionFactor(spinnerConfig.gearReduction)
                     .velocityConversionFactor(spinnerConfig.gearReduction/60);
      masterMotor = MotorControllerFactory.createSparkMax(spinnerConfig.spinnerMasterId, spinnerConfig.spinnerMasterMotorType);
      masterMotor.configure(masterMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    feedbackEncoder = masterMotor.getEncoder();
    if (spinnerConfig.spinnerFollowExists) {
      if (spinnerConfig.spinnerFollowMotorType == MotorConfig.NEO_VORTEX) {
        SparkFlexConfig followFlexConfig = new SparkFlexConfig();
        followFlexConfig.idleMode(IdleMode.kBrake)
                        .inverted(spinnerConfig.spinnerFollowInverted)
                        .follow(spinnerConfig.spinnerMasterId, (spinnerConfig.spinnerMasterInverted != spinnerConfig.spinnerFollowInverted))
                        .encoder.positionConversionFactor(spinnerConfig.gearReduction)
                        .velocityConversionFactor(spinnerConfig.gearReduction/60);
        followMotor = MotorControllerFactory.createSparkFlex(spinnerConfig.spinnerFollowId);
        followMotor.configure(followFlexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      }
      else {
        SparkMaxConfig followMaxConfig = new SparkMaxConfig();
        followMaxConfig.idleMode(IdleMode.kBrake)
                      .follow(spinnerConfig.spinnerMasterId, (spinnerConfig.spinnerMasterInverted != spinnerConfig.spinnerFollowInverted))
                      .inverted(spinnerConfig.spinnerFollowInverted)
                      .encoder.positionConversionFactor(spinnerConfig.gearReduction)
                      .velocityConversionFactor(spinnerConfig.gearReduction/60);
        followMotor = MotorControllerFactory.createSparkMax(spinnerConfig.spinnerFollowId, spinnerConfig.spinnerFollowMotorType);
        followMotor.configure(followMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      }
      backupEncoder = followMotor.getEncoder();
    }
  }

  private void resetFeedbackEncoder() {
    feedbackEncoder.setPosition(0);
    if (spinnerConfig.spinnerFollowExists) {
      backupEncoder.setPosition(0);
    }
  }

  /**
   * Called periodically if in AUTO mode, will approach the current active {@link #spinnerGoal} using PID and FeedForward if such are provided
   */
  protected void autoSpinner() {
    double pidOutput;
    double feedforwardOutput;
    double curVelocity = feedbackEncoder.getVelocity(); 
    if (spinnerConfig.spinnerPIDExists) {
      pidOutput = spinnerConfig.spinnerPID.calculate(curVelocity, spinnerGoal);
    }
    else {
      pidOutput = 0;
    }

    if (spinnerConfig.spinnerFeedForwardExists) {
      feedforwardOutput = spinnerConfig.spinnerFeedforward.calculateWithVelocities(curVelocity, spinnerGoal);
    }
    else {
      feedforwardOutput = 0;
    }
    masterMotor.setVoltage(MathUtil.clamp(pidOutput + feedforwardOutput, -spinnerConfig.maxVolts, 0));
  }

  /**
   * Called periodically if in MANUAL mode, will use {@link #latestManualInput} to spin the spinner using voltage percentage
   */
  protected void manualSpinner() {
      masterMotor.set(MathUtil.clamp(latestManualInput, -spinnerConfig.maxManualInput, spinnerConfig.maxManualInput));
  }

  /**
   * Mainly used by SysId to characterize the spinner model
   * @param volts Voltage unit input
   */
  protected void manualSpinner(Voltage volts) {
    double voltage = volts.in(Volts);
    masterMotor.setVoltage(MathUtil.clamp(voltage, -spinnerConfig.maxManualInput, spinnerConfig.maxManualInput));

  }

  /**
   * Called periodically, checks for any issues with the main encoder by comparing it with the backup encoder if such exists
   */
  protected void checkFeedBackEncoder() {
    if (spinnerConfig.spinnerFollowExists) {
      currentDif = feedbackEncoder.getPosition() - backupEncoder.getPosition();
      if (Math.abs(currentDif) > difDanger && !encoderFaultReported) {
        DriverStation.reportWarning("Spinner encoder seems to be off!", true);
        encoderFaultReported = true;
      }
    }
  }


  /**
   * Sets voltage to 0
   */
  public void stopSpinner() {
    masterMotor.set(0);
  }

  /**
   * Use this to do any fun stuff you want to do, for spinnerSubsystemPeriodic
   */
  protected void userPeriodic() {

  }

  /**
   * Posts important information about Spinner Subsystem into SmartDashboard
   */
  public void postSmartDashboardValues() {
    SmartDashboard.putNumber("Spinner Velocity (rotations/s)", feedbackEncoder.getVelocity());
    SmartDashboard.putNumber("Spinner Goal Velocity (rotations/s)", spinnerGoal);
    SmartDashboard.putBoolean("Spinner at Goal Speed", atGoal());
    SmartDashboard.putNumber("Spinner Latest Manual Input", latestManualInput);
    SmartDashboard.putNumber("Current period of Spinner (ms)", currentPeriod*1000);
    switch (currentState) {
      case AUTO -> SmartDashboard.putString("spinnerControlState", "Autonomous");
      case MANUAL -> SmartDashboard.putString("spinnerControlState", "Manual");
      case BRAKE -> SmartDashboard.putString("spinnerControlState", "Idling: Brake");
      case COAST -> SmartDashboard.putString("spinnerControlState", "Idling: Coast");
    }
  }

   /**
   * Checks Subsystem periodic for running under expected time limits, reports inconsitencies to user, also is usable by feedforward
   */
  private void checkPeriodic() {
    if (pastStamp == 0) {
      pastStamp = Timer.getFPGATimestamp();
      return;
    }
    currentStamp = Timer.getFPGATimestamp();
    currentPeriod = currentStamp - pastStamp;
    pastStamp = currentStamp;
    if (currentPeriod > 0.02) {
      DriverStation.reportWarning("Spinner Periodic is slow: " + currentPeriod *1000 + " ms. Check for any lag reasons", true);
    }
  }

  private void calculateAccleration() {
    double currentVelocity = feedbackEncoder.getVelocity();
    currentAcceleration = (currentVelocity - previousVelocity) / currentPeriod;
    previousVelocity = currentVelocity;
  }

  /*Overide these methods to customize what the mechanism does in different states */
  protected void handleAuto()   { autoSpinner();}
  protected void handleManual() { manualSpinner();}
  protected void handleBrake()  { stopSpinner();}
  protected void handleCoast()  { stopSpinner();}

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
    calculateAccleration();
  }
}
