/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.carlmontrobotics.lib199;

import org.carlmontrobotics.lib199.sim.MockSparkFlex;
import org.carlmontrobotics.lib199.sim.MockSparkMax;
// import org.carlmontrobotics.lib199.sim.MockSparkFlex;
// import org.carlmontrobotics.lib199.sim.MockSparkMax;
import org.carlmontrobotics.lib199.sim.MockTalonSRX;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Add your docs here.
 */
public class MotorControllerFactory {
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
  /**
   * Create a default sparkMax controller (NEO or 550).
   * 
   * @param id the port of the motor controller
   * @param motorConfig either MotorConfig.NEO or MotorConfig.NEO_550
   */
  public static SparkMax createSparkMax(int id, MotorConfig motorConfig) {
    return createSparkMax(id, motorConfig, sparkConfig(motorConfig));
  }
  /**
   * Create a sparkMax controller (NEO or 550) with custom settings.
   * 
   * @param id the port of the motor controller
   * @param config the custom config to set
   */
  public static SparkMax createSparkMax(int id, MotorConfig motorConfig, SparkBaseConfig config) {
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
    MotorErrors.reportSparkTemp(spark, motorConfig.temperatureLimitCelsius);
    MotorErrors.checkSparkErrors(spark);

    return spark;
  }
  /**
   * Create a default SparkFlex-Vortex controller.  
   * 
   * @param id the port of the motor controller
   */
  public static SparkFlex createSparkFlex(int id) {
    return createSparkFlex(id, MotorConfig.NEO_VORTEX, sparkConfig(MotorConfig.NEO_VORTEX));
  }
  /**
   * Create a sparkFlex controller (VORTEX) with custom settings.
   * 
   * @param id the port of the motor controller
   * @param config the custom config to set
   */
  public static SparkFlex createSparkFlex(int id, MotorConfig motorConfig, SparkBaseConfig config) {
    SparkFlex spark = null;
    if (RobotBase.isReal()) {
      spark = new SparkFlex(id, SparkLowLevel.MotorType.kBrushless);
    } else {
      spark = MockSparkFlex.createMockSparkFlex(id, SparkLowLevel.MotorType.kBrushless);
    }

    MotorErrors.reportSparkTemp(spark, motorConfig.temperatureLimitCelsius);
    MotorErrors.checkSparkErrors(spark);

    return spark;
  }

  public static SparkBaseConfig createConfig(MotorControllerType type) {
    SparkBaseConfig config = null;
    switch(type){
      case SPARK_MAX:
        config = new SparkMaxConfig();
        break;
      case SPARK_FLEX:
        config = new SparkFlexConfig();
        break;
    }    
    return config;
  }

  public static MotorControllerType getControllerType(SparkBase motor){
    if(motor instanceof SparkMax){
      return MotorControllerType.SPARK_MAX;
    }else if(motor instanceof SparkFlex){
      return MotorControllerType.SPARK_FLEX;
    }
    return null;
  }

  public static SparkBaseConfig sparkConfig(MotorConfig motorConfig){
    SparkBaseConfig config = motorConfig.controllerType.createConfig();
    //configs that apply to all motors
    config.idleMode(IdleMode.kBrake);
    config.voltageCompensation(12);
    config.smartCurrentLimit(motorConfig.currentLimitAmps);

    config.closedLoop
      .minOutput(-1)
      .maxOutput(1)
      .pid(0,0,0)
      .velocityFF(0);

    return config;
  }
}