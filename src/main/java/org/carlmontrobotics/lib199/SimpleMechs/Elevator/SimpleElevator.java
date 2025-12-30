// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.lib199.SimpleMechs.Elevator;


import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.SimpleMechs.Elastic;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;

public class SimpleElevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final ElevatorConfig elevatorConfig;
  private SparkBase masterMotor;
  private SparkBase followMotor;
  private RelativeEncoder feedbackEncoder;
  private RelativeEncoder backupEncoder;

  private double elevatorGoal;
  private double latestManualInput; // in percentage of power (-1, 1)

  private ElevatorControlState currentState = ElevatorControlState.AUTO;
  private IdleMode elevatorIdleMode = IdleMode.kBrake;

  private double currentDif = 0;
  private final double difDanger = 0.03;

  private double currentStamp;
  private double pastStamp;
  private double currentPeriod = 0.02;

  private boolean encoderFaultReported = false;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance m_distance = Meters.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

  private SysIdRoutine.Config defaultSysIdConfig = new SysIdRoutine.Config(
    Volts.of(1).per(Seconds),//ramp rate, volts/sec
    Volts.of(1), //starting voltage, volts
    Seconds.of(5)//maximum sysID test time
  );


  private final SysIdRoutine sysIdRoutine;

  /**
   * Different control states for the elevator
   * {@link #AUTO} runs the elevator autonomously to go to a certain height
   * {@link #MANUAL} allows the elevator to be manually run by the user with soft locks, and speed limits
   * {@link #BRAKE} idles the elevator in brake, which in cases will cause the elevator to slowly drift downwards due to gravity
   * {@link #COAST} idles the elevator in coast allowing for easy movement of the elevator during disable mode, will likely cause elevator to drop very fast though
   */
  public enum ElevatorControlState {
    AUTO,
    MANUAL,
    BRAKE,
    COAST
  } 

  /**
   * Creates an object of a simple elevator, this class can be extended for more complicated use.
   * @param elevatorConfig {@link ElevatorConfig} for all needs of the elevator
   */
  public SimpleElevator(ElevatorConfig elevatorConfig) {
    this.elevatorConfig = elevatorConfig;
    configureMotors();
    resetFeedbackEncoder();
    sysIdRoutine = new SysIdRoutine(
      defaultSysIdConfig,
      new SysIdRoutine.Mechanism(
        voltage -> {
          manualElevator(voltage);
        },
        log -> {
          log.motor("Elevator-Master")
            .voltage(
              m_appliedVoltage
            .mut_replace(masterMotor.getBusVoltage() * masterMotor.getAppliedOutput(), Volts))
            .linearPosition(m_distance.mut_replace(feedbackEncoder.getPosition(), Meters))
            .linearVelocity(m_velocity.mut_replace(feedbackEncoder.getVelocity(), MetersPerSecond));
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
    Elastic.Notification notification = new Elastic.Notification(Elastic.NotificationLevel.INFO, "SysId Testing On", "Please add Elevator Quasistatic and Dynamics and begin testing.").withDisplaySeconds(5);
    Elastic.sendNotification(notification);
    Elastic.selectTab("SysId");
    SmartDashboard.putData("Elevator Quasistatic Backward", sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("Elevator Quasistatic Forward", sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("Elevator Dynamic Forward", sysIdDynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("Elevator Dynamic Backward", sysIdDynamic(SysIdRoutine.Direction.kReverse));
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
   * Sets {@link #elevatorGoal}
   * @param goal in meters
   */
  public void setGoal(double goal){
    if (goal <= elevatorConfig.topLimit && goal >= elevatorConfig.bottomLimit) {elevatorGoal = goal;}
  }

  /**
   * @return current {@link #elevatorGoal} in meters
   */
  public double getGoal() {
    return elevatorGoal;
  }

  /**
   * Checks if at {@link #elevatorGoal}
   * @return true if at {@link #elevatorGoal}
   */
  public boolean atGoal() {
    return Math.abs(elevatorGoal - feedbackEncoder.getPosition()) <= elevatorConfig.elevatorPIDTolerance;
  }

  /**
   * Updates {@link #latestManualInput} to manually control elevator
   * @param input voltage percentage from -1.0 - 1.0
   */
  public void setManualInput(double input) {
    latestManualInput = input;
  }

  /**
   * @return {@link #latestManualInput} a value between -1.0 - 1.0, inclusive
   */
  public double getLatestManualInput() {
    return latestManualInput;
  }

  /**
  * @return {@link ElevatorConfig} of the object
  */
  public ElevatorConfig getConfig() {
    return elevatorConfig;
  }

  /**
   * @return the current {@link ElevatorControlState}
   */
  public ElevatorControlState getCurrentControlState() {
    return currentState;
  }

  /**
   * Sets {@link #currentState} to the inputed state
   * @param controlState An {@link ElevatorControlState}
   */
  public void setElevatorControlState(ElevatorControlState controlState) {
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
    if (elevatorConfig.elevatorPIDExists || elevatorConfig.elevatorFeedForwardExists) {
      currentState = ElevatorControlState.AUTO;
    }
    else {
      DriverStation.reportWarning("Any sort of autonomous control mode is disabled due to no PID or FeedForward", true);
    }
  }

  /**
   * Sets {@link #currentState} to be in MANUAL mode
   */
  public void setManualOn() {
    currentState = ElevatorControlState.MANUAL;
  }

  /**
   * Sets {@link #currentState} to be in BRAKE mode, also changes the idle mode of the motors to brake.
   */
  public void setBrakeOn() {
    currentState = ElevatorControlState.BRAKE;
    if (elevatorIdleMode == IdleMode.kCoast) {
      elevatorIdleMode = IdleMode.kBrake;
      brakeMotors();

    }
  }

  /**
   * Sets {@link #currentState} to be in COAST mode, also changes the idle mode of the motors to coast.
   */
  public void setCoastOn() {
    currentState = ElevatorControlState.COAST;
    if (elevatorIdleMode == IdleMode.kBrake) {
      elevatorIdleMode = IdleMode.kCoast;
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
    if (elevatorConfig.elevatorFollowMotorType == MotorConfig.NEO_VORTEX) {
      followMotor.configure(flexBrakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    else {
      followMotor.configure(maxBrakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    if (elevatorConfig.elevatorMasterMotorType == MotorConfig.NEO_VORTEX) {
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
    if (elevatorConfig.elevatorFollowMotorType == MotorConfig.NEO_VORTEX) {
      followMotor.configure(flexCoastConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    else {
      followMotor.configure(maxCoastConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    if (elevatorConfig.elevatorMasterMotorType == MotorConfig.NEO_VORTEX) {
      masterMotor.configure(flexCoastConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    else {
      masterMotor.configure(maxCoastConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  /**
   * Called periodically, checks for any issues with the main encoder by comparing it with the backup encoder
   */
  protected void checkFeedBackEncoder() {
    currentDif = feedbackEncoder.getPosition() - backupEncoder.getPosition();
    if (Math.abs(currentDif) > difDanger && !encoderFaultReported) {
      DriverStation.reportWarning("Elevator encoder seems to be off!", true);
      encoderFaultReported = true;
    }
  }

  /**
   * @return current elevator height in meters
   */
  public double getElevatorHeight() {
    return feedbackEncoder.getPosition();
  }

  /**
   * @return current elevator velocity in meters per second
   */
  public double getElevatorVelocity() {
    return feedbackEncoder.getVelocity();
  }


  /**
   * Called in constructor.
   * Configures the master and follow motors in brake mode and makes follow motor to follow the master.
   * Also configures encoders to read in meters, and meters per second for position and velocity respectively
   */
  protected void configureMotors() {
    if (elevatorConfig.elevatorMasterMotorType == MotorConfig.NEO_VORTEX) {
      SparkFlexConfig masterFlexConfig = new SparkFlexConfig();
      masterFlexConfig.idleMode(IdleMode.kBrake)
                      .inverted(elevatorConfig.elevatorMasterInverted)
                      .encoder.positionConversionFactor(elevatorConfig.gearReduction)
                      .velocityConversionFactor(elevatorConfig.gearReduction/60);
      masterMotor = MotorControllerFactory.createSparkFlex(elevatorConfig.elevatorMasterId);
      masterMotor.configure(masterFlexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    else {
      SparkMaxConfig masterMaxConfig = new SparkMaxConfig();
      masterMaxConfig.idleMode(IdleMode.kBrake)
                     .inverted(elevatorConfig.elevatorMasterInverted)
                     .encoder.positionConversionFactor(elevatorConfig.gearReduction)
                     .velocityConversionFactor(elevatorConfig.gearReduction/60);
      masterMotor = MotorControllerFactory.createSparkMax(elevatorConfig.elevatorMasterId, elevatorConfig.elevatorMasterMotorType);
      masterMotor.configure(masterMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    feedbackEncoder = masterMotor.getEncoder();

    if (elevatorConfig.elevatorFollowMotorType == MotorConfig.NEO_VORTEX) {
      SparkFlexConfig followFlexConfig = new SparkFlexConfig();
      followFlexConfig.idleMode(IdleMode.kBrake)
                      .inverted(elevatorConfig.elevatorFollowInverted)
                      .follow(elevatorConfig.elevatorMasterId, (elevatorConfig.elevatorMasterInverted != elevatorConfig.elevatorFollowInverted))
                      .encoder.positionConversionFactor(elevatorConfig.gearReduction)
                      .velocityConversionFactor(elevatorConfig.gearReduction/60);
      followMotor = MotorControllerFactory.createSparkFlex(elevatorConfig.elevatorFollowId);
      followMotor.configure(followFlexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    else {
      SparkMaxConfig followMaxConfig = new SparkMaxConfig();
      followMaxConfig.idleMode(IdleMode.kBrake)
                     .follow(elevatorConfig.elevatorMasterId, (elevatorConfig.elevatorMasterInverted != elevatorConfig.elevatorFollowInverted))
                     .inverted(elevatorConfig.elevatorFollowInverted)
                     .encoder.positionConversionFactor(elevatorConfig.gearReduction)
                     .velocityConversionFactor(elevatorConfig.gearReduction/60);
      followMotor = MotorControllerFactory.createSparkMax(elevatorConfig.elevatorFollowId, elevatorConfig.elevatorFollowMotorType);
      followMotor.configure(followMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    backupEncoder = followMotor.getEncoder();
  }

  /**
   * Called periodically if in AUTO mode, will approach the current active {@link #elevatorGoal} using PID and FeedForward if such are provided
   */
  protected void autoElevator() {
    double pidOutput;
    double feedforwardOutput;
    double curPosition = feedbackEncoder.getPosition(); 
    if (elevatorConfig.elevatorPIDExists) {
      pidOutput = elevatorConfig.elevatorPID.calculate(curPosition, elevatorGoal);
    }
    else {
      pidOutput = 0;
    }

    if (elevatorConfig.elevatorFeedForwardExists) {
      feedforwardOutput = elevatorConfig.elevatorFeedforward.calculate(0); //Voltage to hold at desired spot
    }
    else {
      feedforwardOutput = 0;
    }
    if (curPosition < elevatorConfig.topLimit) {
      if (curPosition > elevatorConfig.bottomLimit) {
        masterMotor.setVoltage(MathUtil.clamp(pidOutput + feedforwardOutput, -elevatorConfig.maxVolts, elevatorConfig.maxVolts));
      }
      else {
        masterMotor.setVoltage(MathUtil.clamp(pidOutput + feedforwardOutput, 0, elevatorConfig.maxVolts));
      }
    }
    else {
      masterMotor.setVoltage(MathUtil.clamp(pidOutput + feedforwardOutput, -elevatorConfig.maxVolts, 0));
    }
  }

  /**
   * Called periodically if in MANUAL mode, will use {@link #latestManualInput} to move the elevator using voltage percentage with soft locking to avoid elevator damage
   */
  protected void manualElevator() {
    double curPosition = feedbackEncoder.getPosition();
    if (curPosition < elevatorConfig.topLimit) {
      if (curPosition > elevatorConfig.bottomLimit) {
        masterMotor.set(MathUtil.clamp(latestManualInput, -elevatorConfig.maxManualInput, elevatorConfig.maxManualInput));
      }
      else {
        masterMotor.set(MathUtil.clamp(latestManualInput, 0, elevatorConfig.maxManualInput));
      }
    }
    else {
      masterMotor.set(MathUtil.clamp(latestManualInput, -elevatorConfig.maxManualInput, 0));
    }
  }

  /**
   * Mainly used by SysId to characterize an elevator model
   * @param volts Voltage unit input
   */
  protected void manualElevator(Voltage volts) {
    double curPosition = feedbackEncoder.getPosition();
    double voltage = volts.in(Volts);
    if (curPosition < elevatorConfig.topLimit) {
      if (curPosition > elevatorConfig.bottomLimit) {
        masterMotor.setVoltage(MathUtil.clamp(voltage, -12, 12));
      }
      else {
        masterMotor.setVoltage(MathUtil.clamp(voltage, 0, 12));
      }
    }
    else {
      masterMotor.set(MathUtil.clamp(voltage,-12, 0));
    }
  }

  /**
   * Sets voltage to 0
   */
  public void stopElevator() {
    masterMotor.set(0);
  }

  /**
   * Called in constructor
   * Resets the feedback encoder to the lowest point the elevator can achieve
   */
  public void resetFeedbackEncoder() {
    resetFeedbackEncoder(elevatorConfig.bottomLimit);
  }

  /**
   * Allow for manual reseting
   * @param position resets to this position in meters
   */
  public void resetFeedbackEncoder(double position) {
    if (position >= elevatorConfig.bottomLimit && position <= elevatorConfig.topLimit) {
      feedbackEncoder.setPosition(position);
      backupEncoder.setPosition(position);
    }
    else {
      DriverStation.reportWarning("Cannot reset position to be outside the limits", true);
    }
  }

  /**
   * Checks boolean suppliers for any bottom or top limit switches to update the feedbackEncoder in case it drifted a bit
   */
  protected void updateFeedbackEncoder() {
    if (elevatorConfig.bottomReset != null) {
      if (elevatorConfig.bottomReset.getAsBoolean()) {
        feedbackEncoder.setPosition(elevatorConfig.bottomLimit);
      }
    }
    if (elevatorConfig.topReset != null) {
      if (elevatorConfig.topReset.getAsBoolean()) {
        feedbackEncoder.setPosition(elevatorConfig.topLimit);
      }
    }
  }

  /**
   * Use this to do any fun stuff you want to do, for elevatorSubsystemPeriodic
   */
  protected void userPeriodic() {

  }

  /**
   * @return current calculated period in seconds
   */
  protected double getCurrentPeriod() {
    return currentPeriod;
  }

  /**
   * Posts important information about elevator into smartdashboard
   */
  protected void postSmartDashboardValues() {
    SmartDashboard.putBoolean("At Goal", atGoal());
    SmartDashboard.putNumber("Goal(m)", elevatorGoal);
    SmartDashboard.putNumber("Elevator Height(m)", getElevatorHeight());
    SmartDashboard.putNumber("Elevator Velocity (m/s)", getElevatorVelocity());
    SmartDashboard.putNumber("Latest manual input", latestManualInput);
    SmartDashboard.putNumber("Current period of Elevator (ms)", currentPeriod*1000);
    switch (currentState) {
      case AUTO -> SmartDashboard.putString("elevatorControlState", "Autonomous");
      case MANUAL -> SmartDashboard.putString("elevatorControlState", "Manual");
      case BRAKE -> SmartDashboard.putString("elevatorControlState", "Idling: Brake");
      case COAST -> SmartDashboard.putString("elevatorControlState", "Idling: Coast");
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
      DriverStation.reportWarning("Elevator Periodic is slow: " + currentPeriod *1000 + " ms. Check for any lag reasons", true);
    }
  }

  /*Overide these methods to customize what the mechanism does in different states */
  protected void handleAuto()   { autoElevator();}
  protected void handleManual() { manualElevator();}
  protected void handleBrake()  { stopElevator();}
  protected void handleCoast()  { stopElevator();}

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
    updateFeedbackEncoder();
    userPeriodic();
    postSmartDashboardValues();
    checkPeriodic();
  }
}
