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
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkClosedLoopController;

// import org.carlmontrobotics.lib199.sim.MockSparkFlex;
// import org.carlmontrobotics.lib199.sim.MockSparkMax;
// FIXME: sorry but we don't need these right now
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
  public static SparkMax createSparkMax(int id, MotorErrors.TemperatureLimit temperatureLimit) {
    return createSparkMax(id, temperatureLimit.limit);
  }

  @Deprecated
  public static SparkMax createSparkMax(int id, int temperatureLimit) {
    SparkMax spark=null;
    if (RobotBase.isReal()) {
      spark = new SparkMax(id, SparkLowLevel.MotorType.kBrushless);
    } else {
      System.err.println("heyy... lib199 doesn't have sim support sorri");
      // spark = MockSparkMax.createMockSparkMax(id, SparkLowLevel.MotorType.kBrushless);
    }
    SparkMaxConfig config = new SparkMaxConfig();
    // config.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus0, 1);
    //FIXME: What is kStatus0
    // config.signals.
    if (spark!=null)
      MotorErrors.reportSparkMaxTemp(spark, temperatureLimit);

    // MotorErrors.reportError(config.follow(ExternalFollower.kFollowerDisabled, 0));
    // config.follow(null, false); dont follow nothing because thats the norm
    // MotorErrors.reportError(config.setIdleMode(IdleMode.kBrake));
    config.idleMode(IdleMode.kBrake);
    // MotorErrors.reportError(config.enableVoltageCompensation(12));
    config.voltageCompensation(12);
    // MotorErrors.reportError(config.smartCurrentLimit(50));
    config.smartCurrentLimit(50);
    
    MotorErrors.checkSparkMaxErrors(spark);
    SparkClosedLoopController controller = spark.getClosedLoopController();
    // MotorErrors.reportError(controller.setOutputRange(-1, 1));
    config.closedLoop.minOutput(-1);
    config.closedLoop.maxOutput(1);
    // MotorErrors.reportError(controller.setP(0));
    // MotorErrors.reportError(controller.setI(0));
    // MotorErrors.reportError(controller.setD(0));
    config.closedLoop.p(0, ClosedLoopSlot.kSlot0);
    config.closedLoop.i(0, ClosedLoopSlot.kSlot0);
    config.closedLoop.d(0, ClosedLoopSlot.kSlot0);
    // MotorErrors.reportError(controller.setFF(0));
    config.closedLoop.velocityFF(0);

    return spark;
  }

  public static SparkMax createSparkMax(int id, SparkBaseConfig config) {
    SparkMax spark = null;
    if (RobotBase.isReal()) {
      spark = new SparkMax(id, SparkLowLevel.MotorType.kBrushless);
    } else {
      System.err.println("heyy... lib199 doesn't have sim support sorri");
      // spark = MockSparkMax.createMockSparkMax(id, SparkLowLevel.MotorType.kBrushless);
    }
    if (spark!=null)
      spark.configure(
        config, 
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters
      );

    return spark;
  }

  public static SparkFlex createSparkFlex(int id, SparkBaseConfig config) {
    SparkFlex spark = null;
    if (RobotBase.isReal()) {
      spark = new SparkFlex(id, SparkLowLevel.MotorType.kBrushless);
    } else {
      System.err.println("heyy... lib199 doesn't have sim support sorri");
      // spark = MockSparkFlex.createMockSparkFlex(id, SparkLowLevel.MotorType.kBrushless);
    }
    if (spark!=null)
      spark.configure(
        config, 
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters
      );

    return spark;
  }

  private static void configureSpark(SparkBase spark, SparkMaxConfig config) {
    MotorErrors.reportSparkTemp(spark, (int) spark.getMotorTemperature());
  
    SparkMaxConfig newConfig = new SparkMaxConfig();

    config.idleMode(IdleMode.kBrake);
    
    config.voltageCompensation(12);
    config.smartCurrentLimit(50);
    
    config.closedLoop.minOutput(-1);
    config.closedLoop.maxOutput(1);
    config.closedLoop.p(0, ClosedLoopSlot.kSlot0);
    config.closedLoop.i(0, ClosedLoopSlot.kSlot0);
    config.closedLoop.d(0, ClosedLoopSlot.kSlot0);
    config.closedLoop.velocityFF(0);

    spark.configure(
      config, 
      SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kNoPersistParameters
    );
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
