/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.carlmontrobotics.lib199;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;

import org.carlmontrobotics.lib199.sim.MockSparkMax;
import org.carlmontrobotics.lib199.sim.MockedCANCoder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Add your docs here.
 */
public class MotorControllerFactory {

  public static TalonFX createTalon(int id) {
    TalonFX talon = new TalonFX(id);

    // Put all configurations for the talon motor controllers in here.
    // All values are from last year's code.
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.PeakForwardDutyCycle = 1;
    motorOutputConfigs.PeakReverseDutyCycle = -1;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    motorOutputConfigs.DutyCycleNeutralDeadband = 0.001;
    MotorErrors.reportError(talon.getConfigurator().apply(motorOutputConfigs));

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    // 40 Amps is the amp limit of a CIM. lThe PDP has 40 amp circuit breakers,
    currentLimits.SupplyCurrentLimit = 30;
    currentLimits.SupplyCurrentThreshold = 0;
    currentLimits.StatorCurrentLimitEnable = true;
    // Why is the peak current limit 0 amps?
    // MotorErrors.reportError(talon.configPeakCurrentLimit(0, 0));
    // MotorErrors.reportError(talon.configPeakCurrentDuration(0, 0));
    MotorErrors.reportError(talon.getConfigurator().apply(currentLimits));

    return talon;
  }

  //checks for spark max errors

  @Deprecated
  public static CANSparkMax createSparkMax(int id, MotorErrors.TemperatureLimit temperatureLimit) {
    return createSparkMax(id, temperatureLimit.limit);
  }

  @Deprecated
  public static CANSparkMax createSparkMax(int id, int temperatureLimit) {
    CANSparkMax spark;
    if (RobotBase.isReal()) {
      spark = new CachedSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);
      if (spark.getFirmwareVersion() == 0) {
        spark.close();
        System.err.println("SparkMax on port: " + id + " is not connected!");
        return MotorErrors.createDummySparkMax();
      }
    } else {
        spark = MockSparkMax.createMockSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);
    }

    MotorErrors.reportSparkMaxTemp(spark, temperatureLimit);

    MotorErrors.reportError(spark.restoreFactoryDefaults());
    MotorErrors.reportError(spark.follow(ExternalFollower.kFollowerDisabled, 0));
    MotorErrors.reportError(spark.setIdleMode(IdleMode.kBrake));
    MotorErrors.reportError(spark.enableVoltageCompensation(12));
    MotorErrors.reportError(spark.setSmartCurrentLimit(50));

    MotorErrors.checkSparkMaxErrors(spark);

    SparkMaxPIDController controller = spark.getPIDController();
    MotorErrors.reportError(controller.setOutputRange(-1, 1));
    MotorErrors.reportError(controller.setP(0));
    MotorErrors.reportError(controller.setI(0));
    MotorErrors.reportError(controller.setD(0));
    MotorErrors.reportError(controller.setFF(0));

    return spark;
  }

  public static CANSparkMax createSparkMax(int id, MotorConfig config) {
    CANSparkMax spark;
    if (RobotBase.isReal()) {
      spark = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);
    } else {
      spark = MockSparkMax.createMockSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);
    }

    MotorErrors.reportSparkMaxTemp(spark, config.temperatureLimitCelsius);

    MotorErrors.reportError(spark.restoreFactoryDefaults());
    MotorErrors.reportError(spark.follow(ExternalFollower.kFollowerDisabled, 0));
    MotorErrors.reportError(spark.setIdleMode(IdleMode.kBrake));
    MotorErrors.reportError(spark.enableVoltageCompensation(12));
    MotorErrors.reportError(spark.setSmartCurrentLimit(config.currentLimitAmps));

    MotorErrors.checkSparkMaxErrors(spark);

    SparkMaxPIDController controller = spark.getPIDController();
    MotorErrors.reportError(controller.setOutputRange(-1, 1));
    MotorErrors.reportError(controller.setP(0));
    MotorErrors.reportError(controller.setI(0));
    MotorErrors.reportError(controller.setD(0));
    MotorErrors.reportError(controller.setFF(0));

    return spark;
  }

  /**
   * @deprecated Use {@link SensorFactory#createCANCoder(int)} instead.
   */
  @Deprecated
  public static CANCoder createCANCoder(int port) {
    CANCoder canCoder = new CANCoder(port);
    if(RobotBase.isSimulation()) new MockedCANCoder(canCoder);
    return canCoder;
  }

  /**
   * Configures a USB Camera.
   * See {@link CameraServer#startAutomaticCapture} for more details.
   * This MUST be called AFTER AHRS initialization or the code will be unable to connect to the gyro.
   *
   * @return The configured camera
   *
   * @deprecated Use {@link SensorFactory#configureCamera()} instead.
   */
  @Deprecated
  public static UsbCamera configureCamera() {
    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    CameraServer.getServer().setSource(camera);
    return camera;
  }

  /**
   * This method is equivalent to calling {@link #configureCamera()} {@code numCameras} times.
   * The last camera will be set as the primary Camera feed.
   * To change it, call {@code CameraServer.getServer().setSource()}.
   *
   * @param numCameras The number of cameras to configure
   * @return The configured cameras.
   *
   * @deprecated Use {@link SensorFactory#configureCameras(int)} instead.
   */
  @Deprecated
  public static UsbCamera[] configureCameras(int numCameras) {
    UsbCamera[] cameras = new UsbCamera[numCameras];
    for(int i = 0; i < numCameras; i++) cameras[i] = configureCamera();
    return cameras;
  }
}