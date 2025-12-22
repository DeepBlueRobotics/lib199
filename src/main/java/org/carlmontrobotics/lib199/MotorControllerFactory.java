/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.carlmontrobotics.lib199;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkClosedLoopController;

import org.carlmontrobotics.lib199.sim.MockSparkFlex;
import org.carlmontrobotics.lib199.sim.MockSparkMax;
// import org.carlmontrobotics.lib199.sim.MockSparkFlex;
// import org.carlmontrobotics.lib199.sim.MockSparkMax;
import org.carlmontrobotics.lib199.sim.MockTalonSRX;
import org.carlmontrobotics.lib199.sim.MockVictorSPX;
import org.carlmontrobotics.lib199.sim.MockedCANCoder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Add your docs here.
 */
public class MotorControllerFactory {
  @Deprecated
  /**
   * @deprecated VictorSPX motor controllers are no longer legal for the 2026 season: https://community.firstinspires.org/2025-robot-rules-preview-for-2026
   */
  public static WPI_VictorSPX createVictor(int port) {
    WPI_VictorSPX victor;
    if (RobotBase.isReal()) {
        victor = new WPI_VictorSPX(port);
    } else {
        victor = MockVictorSPX.createMockVictorSPX(port);
    }

    // Put all configurations for the victor motor controllers in here.
    MotorErrors.reportError(victor.configNominalOutputForward(0, 10));
    MotorErrors.reportError(victor.configNominalOutputReverse(0, 10));
    MotorErrors.reportError(victor.configPeakOutputForward(1, 10));
    MotorErrors.reportError(victor.configPeakOutputReverse(-1, 10));
    MotorErrors.reportError(victor.configNeutralDeadband(0.001, 10));
    victor.setNeutralMode(NeutralMode.Brake);

    return victor;
  }

  public static WPI_TalonSRX createTalon(int id) {
    WPI_TalonSRX talon;
    if (RobotBase.isReal()) {
        talon = new WPI_TalonSRX(id);
    } else {
        talon = MockTalonSRX.createMockTalonSRX(id);
    }

    // Put all configurations for the talon motor controllers in here.
    // All values are from last year's code.
    MotorErrors.reportError(talon.configNominalOutputForward(0, 10));
    MotorErrors.reportError(talon.configNominalOutputReverse(0, 10));
    MotorErrors.reportError(talon.configPeakOutputForward(1, 10));
    MotorErrors.reportError(talon.configPeakOutputReverse(-1, 10));
    MotorErrors.reportError(talon.configPeakCurrentLimit(0, 0));
    MotorErrors.reportError(talon.configPeakCurrentDuration(0, 0));
    // 40 Amps is the amp limit of a CIM. lThe PDP has 40 amp circuit breakers,
    MotorErrors.reportError(talon.configContinuousCurrentLimit(30, 0));
    talon.enableCurrentLimit(true);
    MotorErrors.reportError(talon.configNeutralDeadband(0.001, 10));
    talon.setNeutralMode(NeutralMode.Brake);

    return talon;
  }

  @Deprecated
  /**
   * @deprecated Use {@link MotorControllerFactory#createSparkMax(int id, MotorConfig motorConfig)} instead.
   * Create a default sparkMax controller (NEO or 550).
   * 
   * @param id the port of the motor controller
   * @param motorConfig either MotorConfig.NEO or MotorConfig.NEO_550
   */
  public static SparkMax createSparkMax(int id, MotorConfig motorConfig) {
    if (motorConfig.temperatureLimitCelsius == MotorConfig.NEO.temperatureLimitCelsius) {
      return createSparkMax(id, sparkConfig(SparkMotorType.NEO));
    }else{
      return createSparkMax(id, sparkConfig(SparkMotorType.NEO550));
    }
  }
  /**
   * Create a sparkMax controller (NEO or 550) with custom settings.
   * 
   * @param id the port of the motor controller
   * @param config the custom config to set
   */
  public static SparkMax createSparkMax(int id, SparkBaseConfig config) {
    SparkMax spark;
    if (RobotBase.isReal()) {
      spark = new SparkMax(id, SparkLowLevel.MotorType.kBrushless);
    } else {
      spark = MockSparkMax.createMockSparkMax(id, SparkLowLevel.MotorType.kBrushless, MockSparkMax.NEOType.NEO);
    }
    spark.configure(
      config, 
      SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kNoPersistParameters
    );

    return spark;
  }
  /**
   * Create a default SparkFlex-Vortex controller.  
   * 
   * @param id the port of the motor controller
   */
  public static SparkFlex createSparkFlex(int id) {
    return createSparkFlex(id, sparkConfig(SparkMotorType.VORTEX));
  }
  /**
   * Create a sparkFlex controller (VORTEX) with custom settings.
   * 
   * @param id the port of the motor controller
   * @param config the custom config to set
   */
  public static SparkFlex createSparkFlex(int id, SparkBaseConfig config) {
    SparkFlex spark = null;
    if (RobotBase.isReal()) {
      spark = new SparkFlex(id, SparkLowLevel.MotorType.kBrushless);
    } else {
      spark = MockSparkFlex.createMockSparkFlex(id, SparkLowLevel.MotorType.kBrushless);
    }
    if (spark!=null)
      spark.configure(
        config, 
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      );

    return spark;
  }

  public static SparkBaseConfig sparkConfig(SparkMotorType motor){
    SparkBaseConfig config = null;
    switch(motor.getControllerType()){
      case SPARK_MAX:
        config = new SparkMaxConfig();
        break;
      case SPARK_FLEX:
        config = new SparkFlexConfig();
        break;
    }
    //configs that apply to all motors
    config.idleMode(IdleMode.kBrake);
    config.voltageCompensation(12);
    config.smartCurrentLimit(40); //40 amps is the fuse rating for fuses for each individual motor on the PDP

    config.closedLoop
      .minOutput(-1)
      .maxOutput(1)
      .pid(0,0,0)
      .velocityFF(0);


    //motor specific configs
    switch(motor){
      case NEO:
        break;
      case NEO550:
        config.smartCurrentLimit(20); //so motor no go smoky
        break;
      case VORTEX, SOLO_VORTEX: // the config for a vortex should be the same if it uses a spark max with a solo adapter or a spark flex, so I just combined them together
        break;
      case NEO_2:
        break;
    }

    return config;
  }
}