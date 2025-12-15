// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.lib199.SimpleMechs.Arm;


import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleArm extends SubsystemBase {
  /** Creates a new SimpleArm. */
  ArmConfig armConfig;
  SparkBase armMasterMotor;
  RelativeEncoder armFeedbackEncoder;
  AbsoluteEncoder armResetEncoder;
  RelativeEncoder armBackupEncoder;
  SparkBase[] armFollowMotors;
  double currentDif = 0;
  double pastDif = 0;
  final double difDanger = 1;

  double armGoal; // in degrees
  double latestManualInput; // in percentage of power (-1, 1)

  ArmControlState currentState = ArmControlState.AUTO;
  IdleMode armIdleMode = IdleMode.kBrake;

  Timer armTimer = new Timer();

  private final double defaultRelativeEncoderResetValue = -90;

  public enum ArmControlState {
    AUTO,
    MANUAL,
    BRAKE,
    COAST

  }

  public SimpleArm(ArmConfig armConfig) {
    this.armConfig = armConfig;
    configMotors();
    resetFeedbackEncoder();
  }

  public void setGoal(double goal){
    if (goal <= armConfig.topLimit && goal >= armConfig.bottomLimit) {armGoal = goal;}
  }

  public void setManualInput(double input) {
    latestManualInput = input;
  }

  public void toggleControlMode() {
    if (currentState == ArmControlState.AUTO) {
      currentState = ArmControlState.MANUAL;
    }
    else if (currentState == ArmControlState.MANUAL) {
      currentState = ArmControlState.AUTO;
    }
  }

  public void setArmControlState(ArmControlState controlState) {
    switch (controlState) {
      case AUTO -> setAdjustingOn();
      case MANUAL -> setManualOn();
      case BRAKE -> setBrakeOn();
      case COAST -> setCoastOn();
      default -> System.out.println("Such control mode has not been implemented yet");
    }
  }

  public void setAdjustingOn() {
    if (armConfig.armPIDExists || armConfig.armFeedForwardExists) {
      currentState = ArmControlState.AUTO;
    }
    else {
      System.out.println("Any sort of autonomous control mode is disabled due to no PID or FeedForward");
    }
  }

  public void setManualOn() {
    currentState = ArmControlState.MANUAL;
  }

  public void setBrakeOn() {
    currentState = ArmControlState.BRAKE;
    if (armIdleMode == IdleMode.kCoast) {
      armIdleMode = IdleMode.kBrake;
      brakeMotors();

    }
  }

  public void setCoastOn() {
    currentState = ArmControlState.COAST;
    if (armIdleMode == IdleMode.kBrake) {
      armIdleMode = IdleMode.kCoast;
      coastMotors();
    }
  }

  private void resetFeedbackEncoder() {
    if (armConfig.armMainAbsoluteEncoderExists) {
      armFeedbackEncoder.setPosition((armResetEncoder.getPosition() + armConfig.armAbsoluteZeroOffset)/armConfig.armAbsoluteGearReduction*armConfig.armRelativeGearReduction);
    }
    else {
      armFeedbackEncoder.setPosition(defaultRelativeEncoderResetValue);
    }
  }

  private void checkFeedBackEncoder() {
    pastDif = currentDif;
    currentDif = armFeedbackEncoder.getPosition() - armBackupEncoder.getPosition();
    if (currentDif - pastDif > difDanger) {
      System.out.println("Arm encoder seems to be off!");
    }
  }
    
  
  private void autoArm() {
    double pidOutput;
    double feedforwardOutput;
    double curPosition = armFeedbackEncoder.getPosition();
    if (armConfig.armPIDExists) {
      pidOutput = armConfig.armPID.calculate(curPosition, armGoal);
    }
    else {
      pidOutput = 0;
    }

    if (armConfig.armFeedForwardExists) {
      feedforwardOutput = armConfig.armFeedForward.calculate(armGoal/360*2*Math.PI, 0);
    }
    else {
      feedforwardOutput = 0;
    }
    if (curPosition < armConfig.topLimit) {
      if (curPosition > armConfig.bottomLimit) {
        armMasterMotor.set(MathUtil.clamp(pidOutput + feedforwardOutput, -armConfig.armMaxVolts, armConfig.armMaxVolts));
      }
      else {
        armMasterMotor.set(MathUtil.clamp(pidOutput + feedforwardOutput, 0, armConfig.armMaxVolts));
      }
    }
    else {
      armMasterMotor.set(MathUtil.clamp(pidOutput + feedforwardOutput, -armConfig.armMaxVolts, 0));
    }
  }

  private void manualArm() {
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

  private void configMotors() {
    configFollowMotors();
    configMasterMotor();
  }

  private void configFollowMotors() {
    if (armConfig.armFollowId != null) {
      armFollowMotors = new SparkBase[armConfig.armFollowId.length];

      SparkFlexConfig flexFollowConfig = new SparkFlexConfig();
      flexFollowConfig.idleMode(IdleMode.kBrake);
      SparkMaxConfig maxFollowConfig = new SparkMaxConfig();
      maxFollowConfig.idleMode(IdleMode.kBrake);

      for (int i = 0; i < armConfig.armFollowId.length; i++) {
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

  private void configMasterMotor() {
    if (armConfig.armMasterMotorType == MotorConfig.NEO_VORTEX) {
      SparkFlexConfig flexMasterConfig = new SparkFlexConfig();
      flexMasterConfig.inverted(armConfig.armMasterInversed);
      flexMasterConfig.idleMode(IdleMode.kBrake);
      armMasterMotor = MotorControllerFactory.createSparkFlex(armConfig.armMasterMotorId);
      armMasterMotor.configure(flexMasterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    else {
      SparkMaxConfig maxMasterConfig = new SparkMaxConfig();
      maxMasterConfig.inverted(armConfig.armMasterInversed);
      maxMasterConfig.idleMode(IdleMode.kBrake);
      armMasterMotor = MotorControllerFactory.createSparkMax(armConfig.armMasterMotorId, armConfig.armMasterMotorType);
      armMasterMotor.configure(maxMasterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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

  /**
   * Use this to do any fun stuff you want to do, for armSubsystemPeriodic
   */
  private void userPeriodic() {

  }

  private void postSmartDashboardValues() {
    SmartDashboard.putData(this);
    SmartDashboard.putNumber("armVelocity", armFeedbackEncoder.getVelocity());
    SmartDashboard.putNumber("armPositionDegrees", armFeedbackEncoder.getPosition()); //From vertical down 0
    switch (currentState) {
      case AUTO -> SmartDashboard.putString("armControlState", "Autonomous");
      case MANUAL -> SmartDashboard.putString("armControlState", "Manual");
      case BRAKE -> SmartDashboard.putString("armControlState", "Idling: Brake");
      case COAST -> SmartDashboard.putString("armControlState", "Idling: Coast");
    }
    SmartDashboard.putNumber("armGoal", armGoal);

  }

  /** 
   * DO NOT OVERRIDE THIS PERIODIC UNLESS YOU KNOW WHAT YOU ARE DOING!
   * Use {@link #userPeriodic()} instead.
   */
  @Override
  public void periodic() {
    if (currentState == ArmControlState.AUTO) {autoArm();}
    else if (currentState == ArmControlState.MANUAL) {manualArm();}
    checkFeedBackEncoder();
    userPeriodic();
    postSmartDashboardValues();
  }

}
