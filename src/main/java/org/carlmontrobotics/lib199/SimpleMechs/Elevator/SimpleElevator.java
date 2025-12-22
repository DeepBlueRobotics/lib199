// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.lib199.SimpleMechs.Elevator;


import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.SimpleMechs.Arm.SimpleArm.ArmControlState;

import java.nio.channels.Pipe;

import javax.management.relation.Relation;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleElevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private ElevatorConfig elevatorConfig;
  private SparkBase masterMotor;
  private SparkBase followMotor;
  private RelativeEncoder feedbackEncoder;
  private RelativeEncoder backupEncoder;

  private double elevatorGoal;
  private double latestManualInput; // in percentage of power (-1, 1)

  private ElevatorControlState currentState = ElevatorControlState.AUTO;
  private IdleMode elevatorIdleMode = IdleMode.kBrake;

  private double currentDif = 0;
  private double pastDif = 0;
  private final double difDanger = 1;

  public enum ElevatorControlState {
    AUTO,
    MANUAL,
    BRAKE,
    COAST
  } 

  public SimpleElevator(ElevatorConfig elevatorConfig) {
    this.elevatorConfig = elevatorConfig;
    configureMotors();
    resetFeedbackEncoder();
  }

  public void setGoal(double goal){
    if (goal <= elevatorConfig.topLimit && goal >= elevatorConfig.bottomLimit) {elevatorGoal = goal;}
  }

  public double getGoal() {
    return elevatorGoal;
  }

  public void setManualInput(double input) {
    latestManualInput = input;
  }

  public double getLatestManualInput() {
    return latestManualInput;
  }

  public void setElevatorControlState(ArmControlState controlState) {
    switch (controlState) {
      case AUTO -> setAutoOn();
      case MANUAL -> setManualOn();
      case BRAKE -> setBrakeOn();
      case COAST -> setCoastOn();
      default -> DriverStation.reportWarning("Such control mode has not been implemented yet", true);
    }
  }

  public void setAutoOn() {
    if (elevatorConfig.elevatorPIDExists || elevatorConfig.elevatorFeedForwardExists) {
      currentState = ElevatorControlState.AUTO;
    }
    else {
      DriverStation.reportWarning("Any sort of autonomous control mode is disabled due to no PID or FeedForward", true);
    }
  }

  public void setManualOn() {
    currentState = ElevatorControlState.MANUAL;
  }

  public void setBrakeOn() {
    currentState = ElevatorControlState.BRAKE;
    if (elevatorIdleMode == IdleMode.kCoast) {
      elevatorIdleMode = IdleMode.kBrake;
      brakeMotors();

    }
  }

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

  private void checkFeedBackEncoder() {
    pastDif = currentDif;
    currentDif = feedbackEncoder.getPosition() - backupEncoder.getPosition();
    if (currentDif - pastDif > difDanger) {
      DriverStation.reportWarning("Elevator encoder seems to be off!", true);
    }
  }



  public double getElevatorHeight() {
    return feedbackEncoder.getPosition();
  }

  public double getElevatorVelocity() {
    return feedbackEncoder.getVelocity();
  }



  private void configureMotors() {
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

  private void autoElevator() {
    //TODO
  }

  private void manualElevator() {
    //TODO
  }

  public void stopElevator() {
    masterMotor.set(0);
  }

  public void resetFeedbackEncoder() {
    resetFeedbackEncoder(elevatorConfig.bottomLimit);
  }

  public void resetFeedbackEncoder(double position) {
    if (position >= elevatorConfig.bottomLimit && position <= elevatorConfig.topLimit) {
      feedbackEncoder.setPosition(position);
    }
    else {
      DriverStation.reportWarning("Cannot reset position to be outside the limits", true);
    }
  }

  private void updateFeedbackEncoder() {
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
  public void userPeriodic() {

  }

  private void postSmartDashboardValues() {

  }

  /** 
   * DO NOT OVERRIDE THIS PERIODIC UNLESS YOU KNOW WHAT YOU ARE DOING!
   * Use {@link #userPeriodic()} instead.
   */
  @Override
  public void periodic() {
    switch (currentState) {
      case AUTO -> autoElevator();
      case MANUAL -> manualElevator();
      case BRAKE -> stopElevator();
      case COAST -> stopElevator();
    }
    checkFeedBackEncoder();
    updateFeedbackEncoder();
    userPeriodic();
    postSmartDashboardValues();
  }
}
