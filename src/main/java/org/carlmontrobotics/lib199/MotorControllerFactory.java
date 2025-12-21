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

  /**
   * Create a default sparkMax controller (NEO or 550).
   * 
   * @param id the port of the motor controller
   * @param motorConfig either MotorConfig.NEO or MotorConfig.NEO_550
   */
  public static SparkMax createSparkMax(int id, MotorConfig motorConfig) {
    SparkMax spark=null;
    if (RobotBase.isReal()) {
      spark = new SparkMax(id, SparkLowLevel.MotorType.kBrushless);
    } else {
      spark = MockSparkMax.createMockSparkMax(id, SparkLowLevel.MotorType.kBrushless, MockSparkMax.NEOType.NEO);
    }

    // config.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus0, 1);
    if (spark!=null)
      MotorErrors.reportSparkMaxTemp(spark, motorConfig.temperatureLimitCelsius);
    
    MotorErrors.checkSparkMaxErrors(spark);

    if (motorConfig==MotorConfig.NEO || motorConfig==MotorConfig.NEO_550)
      spark.configure(baseSparkMaxConfig(), SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    else if (motorConfig==MotorConfig.NEO_VORTEX)
      spark.configure(baseSparkFlexConfig(), SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    else
      spark.configure(baseSparkConfig(), SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


    return spark;
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
      spark = MockSparkMax.createMockSparkMax(id, SparkLowLevel.MotorType.kBrushless, MockSparkMax.NEOType.NEO); //what if the Mocked motor is a 550 with a custom config?
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
    MotorConfig motorConfig = MotorConfig.NEO_VORTEX;

    SparkFlex spark;
    if (RobotBase.isReal()) {
      spark = new SparkFlex(id, SparkLowLevel.MotorType.kBrushless);
    } else {
      spark = MockSparkFlex.createMockSparkFlex(id, MotorType.kBrushless);
    }

    // config.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus0, 1);
    MotorErrors.reportSparkTemp(spark, motorConfig.temperatureLimitCelsius);
    
    MotorErrors.checkSparkErrors(spark);

    spark.configure(baseSparkFlexConfig(), SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    return spark;
  }
  /**
   * Create a sparkFlex controller (VORTEX) with custom settings.
   * 
   * @param id the port of the motor controller
   * @param config the custom config to set
   */
  public static SparkFlex createSparkFlex(int id, SparkFlexConfig config) {
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

  //does this not do the same as baseSparkMaxConfig, as it also creates a Spark MaxConfig?
  public static SparkBaseConfig baseSparkConfig() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(IdleMode.kBrake);
    
    config.voltageCompensation(12);
    config.smartCurrentLimit(50);
    
    config.closedLoop
      .minOutput(-1)
      .maxOutput(1)
      .pid(0,0,0)
      .velocityFF(0);

    return config;
  }
  /**
   * Overrides an old config - but does not change other settings.
   */
  //does this not do the same as baseSparkMaxConfig, as it also creates a Spark MaxConfig?
  public static SparkBaseConfig baseSparkConfig(SparkMaxConfig config) {
    config.idleMode(IdleMode.kBrake);
    
    config.voltageCompensation(12);
    config.smartCurrentLimit(50);
    
    config.closedLoop
      .minOutput(-1)
      .maxOutput(1)
      .pid(0,0,0)
      .velocityFF(0);

    return config;
  }
  /**
   * Overrides an old config - but does not change other settings.
   */
  public static SparkMaxConfig baseSparkMaxConfig(SparkMaxConfig config){
    //typical operating voltage: 12V.
    return (SparkMaxConfig) baseSparkConfig(config);//FIXME apply needed config changes for each controller
  }
  public static SparkMaxConfig baseSparkMaxConfig(){
    return (SparkMaxConfig) baseSparkConfig();
  }
  /**
   * Overrides an old config - but does not change other settings.
   */
  public static SparkFlexConfig baseSparkFlexConfig(SparkFlexConfig config){
    //typical operating voltage: 12V. ( same as sparkMax )
    config.idleMode(IdleMode.kBrake);
    
    config.voltageCompensation(12);
    config.smartCurrentLimit(50);
    
    config.closedLoop
      .minOutput(-1)
      .maxOutput(1)
      .pid(0,0,0)
      .velocityFF(0);

    return config;
  }
  public static SparkFlexConfig baseSparkFlexConfig(){//why? no Se.
    SparkFlexConfig config = new SparkFlexConfig();

    config.idleMode(IdleMode.kBrake);
    
    config.voltageCompensation(12);
    config.smartCurrentLimit(50);
    
    config.closedLoop
      .minOutput(-1)
      .maxOutput(1)
      .pid(0,0,0)
      .velocityFF(0);

    return config;
  }
}