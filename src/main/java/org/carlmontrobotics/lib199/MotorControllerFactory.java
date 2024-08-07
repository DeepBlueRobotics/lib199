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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ExternalFollower;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;

import org.carlmontrobotics.lib199.sim.MockSparkFlex;
import org.carlmontrobotics.lib199.sim.MockSparkMax;
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

  //checks for spark max errors

  @Deprecated
  public static CANSparkMax createSparkMax(int id, MotorErrors.TemperatureLimit temperatureLimit) {
    return createSparkMax(id, temperatureLimit.limit);
  }

  @Deprecated
  public static CANSparkMax createSparkMax(int id, int temperatureLimit) {
    CANSparkMax spark;
    if (RobotBase.isReal()) {
      spark = new CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless);
    } else {
        spark = MockSparkMax.createMockSparkMax(id, CANSparkLowLevel.MotorType.kBrushless);
    }
    spark.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 1);

    MotorErrors.reportSparkMaxTemp(spark, temperatureLimit);

    MotorErrors.reportError(spark.restoreFactoryDefaults());
    MotorErrors.reportError(spark.follow(ExternalFollower.kFollowerDisabled, 0));
    MotorErrors.reportError(spark.setIdleMode(IdleMode.kBrake));
    MotorErrors.reportError(spark.enableVoltageCompensation(12));
    MotorErrors.reportError(spark.setSmartCurrentLimit(50));

    MotorErrors.checkSparkMaxErrors(spark);

    SparkPIDController controller = spark.getPIDController();
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
      spark = new CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless);
    } else {
      spark = MockSparkMax.createMockSparkMax(id, CANSparkLowLevel.MotorType.kBrushless);
    }

    configureSpark(spark, config);

    return spark;
  }

  public static CANSparkFlex createSparkFlex(int id, MotorConfig config) {
    CANSparkFlex spark;
    if (RobotBase.isReal()) {
      spark = new CANSparkFlex(id, CANSparkLowLevel.MotorType.kBrushless);
    } else {
      spark = MockSparkFlex.createMockSparkFlex(id, CANSparkLowLevel.MotorType.kBrushless);
    }

    configureSpark(spark, config);

    return spark;
  }

  private static void configureSpark(CANSparkBase spark, MotorConfig config) {
    MotorErrors.reportSparkTemp(spark, config.temperatureLimitCelsius);

    MotorErrors.reportError(spark.restoreFactoryDefaults());
    //MotorErrors.reportError(spark.follow(ExternalFollower.kFollowerDisabled, 0));
    MotorErrors.reportError(spark.setIdleMode(IdleMode.kBrake));
    MotorErrors.reportError(spark.enableVoltageCompensation(12));
    MotorErrors.reportError(spark.setSmartCurrentLimit(config.currentLimitAmps));

    MotorErrors.checkSparkErrors(spark);

    SparkPIDController controller = spark.getPIDController();
    MotorErrors.reportError(controller.setOutputRange(-1, 1));
    MotorErrors.reportError(controller.setP(0));
    MotorErrors.reportError(controller.setI(0));
    MotorErrors.reportError(controller.setD(0));
    MotorErrors.reportError(controller.setFF(0));
  }

  /**
   * @deprecated Use {@link SensorFactory#createCANCoder(int)} instead.
   */
  @Deprecated
  public static CANcoder createCANCoder(int port) {
    CANcoder canCoder = new CANcoder(port);
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
