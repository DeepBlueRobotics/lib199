package org.carlmontrobotics.lib199.SimpleMechs.Drivetrain;
import java.util.Arrays;
import java.util.Map;
import java.util.function.Supplier;

import javax.print.attribute.standard.PrinterInfo;

import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.SensorFactory;
import org.carlmontrobotics.lib199.swerve.SwerveConfig;
import org.carlmontrobotics.lib199.swerve.SwerveModule;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.CANcoder;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import org.carlmontrobotics.lib199.swerve.SwerveModuleSim;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

public class SimpleDrivetrain extends SubsystemBase {

    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    private Pose2d autoGyroOffset = new Pose2d(0., 0., new Rotation2d(0.));
    // ^used by PathPlanner for chaining paths
    private SwerveDriveKinematics kinematics = null;
    private SwerveDriveOdometry odometry = null;
    private SwerveDrivePoseEstimator poseEstimator = null;

    private SwerveModule modules[];
    private boolean fieldOriented = true;
    private double fieldOffset = 0;

    private SparkFlex[] flexDriveMotors = new SparkFlex[] { null, null, null, null };
    private SparkFlex[] flexTurnMotors = new SparkFlex[] { null, null, null, null };
    private SparkMax[] maxDriveMotors = new SparkMax[] { null, null, null, null };
    private SparkMax[] maxTurnMotors = new SparkMax[] { null, null, null, null };
    private CANcoder[] turnEncoders = new CANcoder[] { null, null, null, null };
    private final SparkClosedLoopController[] turnPidControllers = new SparkClosedLoopController[] {null, null, null, null};
    public final float initPitch;
    public final float initRoll;

    // debug purposes
    private SwerveConfig swerveConfig;
    private SwerveModule moduleFL;
    private SwerveModule moduleFR;
    private SwerveModule moduleBL;
    private SwerveModule moduleBR;

    private final Field2d field = new Field2d();
    private final Field2d odometryField = new Field2d();
    private final Field2d poseWithLimelightField = new Field2d();

    public double ppKpDrive = 5.0;
    public double ppKiDrive = 0;
    public double ppKdDrive = 0;

    public double ppKpTurn = 3;
    public double ppKiTurn = 0;
    public double ppKdTurn = 0;

    double accelX;
    double accelY;
    double accelXY;

    private SwerveModuleSim[] moduleSims;
    private SimDouble gyroYawSim;
    private Timer simTimer = new Timer();
    public double extraSpeedMult = 0;

    private double lastSetX = 0, lastSetY = 0, lastSetTheta = 0;
    double kP = 0;
    double kI = 0;
    double kD = 0;
    public enum Mode {
    coast,
    brake,
    toggle
    }

    //Constants
    private double g = 9.8; //m/s^2
    private double mu = 1;
    private double autoCentripetalAccel = mu * g * 2;


    //Extra
    private double wheelBase;
    private double trackWidth;

    private int driveFrontLeftPort;
    private int turnFrontLeftPort;
    private int canCoderPortFL;

    private int driveFrontRightPort;
    private int turnFrontRightPort;
    private int canCoderPortFR;
    
    private int driveBackLeftPort;
    private int turnBackLeftPort;
    private int canCoderPortBL;

    private int driveBackRightPort;
    private int turnBackRightPort;
    private int canCoderPortBR;

    private double secsPer12Volts;
    private double wheelDiameterMeters;
    private double driveGearing;
    private double turnGearing;

    private boolean isGyroReversed;
    private double maxSpeed;

    private RobotConfig robotConfig;

    private double COLLISION_ACCELERATION_THRESHOLD;

    private boolean setupSysId;

    private boolean useFlexDrive;
    private boolean useFlexTurn;

    private Rotation2d simGyroOffset = new Rotation2d();

    

    /**
     * 
     * @param wheelBase in meters
     * @param trackWidth in meters
     */
    public SimpleDrivetrain(
        SwerveConfig swerveConfig,
        double wheelBase, double trackWidth,
        int driveFrontLeftPort, int turnFrontLeftPort, int canCoderPortFL,
        int driveFrontRightPort, int turnFrontRightPort, int canCoderPortFR,
        int driveBackLeftPort, int turnBackLeftPort, int canCoderPortBL,
        int driveBackRightPort, int turnBackRightPort, int canCoderPortBR,
        double secsPer12Volts, double wheelDiameterMeters, double driveGearing, double turnGearing,
        boolean isGyroReversed, double maxSpeed,
        RobotConfig robotConfig,
        double COLLISION_ACCELERATION_THRESHOLD,
        boolean useFlexDrive,
        boolean useFlexTurn
    ) 
    {
        this.swerveConfig = swerveConfig;
        this.wheelBase = wheelBase;
        this.trackWidth = trackWidth;
        this.driveFrontLeftPort = driveFrontLeftPort;
        this.turnFrontLeftPort = turnFrontLeftPort;
        this.canCoderPortFL = canCoderPortFL;
        this.driveFrontRightPort = driveFrontRightPort;
        this.turnFrontRightPort = turnFrontRightPort;
        this.canCoderPortFR = canCoderPortFR;
        this.driveBackLeftPort = driveBackLeftPort;
        this.turnBackLeftPort = turnBackLeftPort;
        this.canCoderPortBL = canCoderPortBL;
        this.driveBackRightPort = driveBackRightPort;
        this.turnBackRightPort = turnBackRightPort;
        this.canCoderPortBR = canCoderPortBR;
        this.secsPer12Volts = secsPer12Volts;
        this.wheelDiameterMeters = wheelDiameterMeters;
        this.driveGearing = driveGearing;
        this.turnGearing = turnGearing;
        this.isGyroReversed = isGyroReversed;
        this.maxSpeed = maxSpeed;
        this.robotConfig = robotConfig;
        this.COLLISION_ACCELERATION_THRESHOLD = COLLISION_ACCELERATION_THRESHOLD;
        this.useFlexDrive = useFlexDrive;
        this.useFlexTurn = useFlexDrive;

        AutoBuilder();

        double initTimestamp = Timer.getFPGATimestamp();
        double currentTimestamp = initTimestamp;
        while (gyro.isCalibrating() && currentTimestamp - initTimestamp < 10) {
            currentTimestamp = Timer.getFPGATimestamp();
            try {
                Thread.sleep(1000);// 1 second
            } catch (InterruptedException e) {
                e.printStackTrace();
                break;
            }
            System.out.println("Calibrating the gyro...");
        }
        gyro.reset();
        // this.resetFieldOrientation();
        System.out.println("NavX-MXP firmware version: " + gyro.getFirmwareVersion());
        System.out.println("Magnetometer is calibrated: " + gyro.isMagnetometerCalibrated());

        // Setup Kinematics
        {
            // Define the corners of the robot relative to the center of the robot using
            // Translation2d objects.
            // Positive x-values represent moving toward the front of the robot whereas
            // positive y-values represent moving toward the left of the robot.
            Translation2d locationFL = new Translation2d(wheelBase / 2, trackWidth / 2);
            Translation2d locationFR = new Translation2d(wheelBase / 2, -trackWidth / 2);
            Translation2d locationBL = new Translation2d(-wheelBase / 2, trackWidth / 2);
            Translation2d locationBR = new Translation2d(-wheelBase / 2, -trackWidth / 2);

            kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);
        }

        // Initialize modules
        {
            // initPitch = 0;
            // initRoll = 0;
            Supplier<Float> pitchSupplier = () -> 0F;
            Supplier<Float> rollSupplier = () -> 0F;
            initPitch = gyro.getPitch();
            initRoll = gyro.getRoll();
            // Supplier<Float> pitchSupplier = () -> gyro.getPitch();
            // Supplier<Float> rollSupplier = () -> gyro.getRoll();

            if (useFlexDrive) {
                if (useFlexTurn) {
                    moduleFL = new SwerveModule(swerveConfig, SwerveModule.ModuleType.FL, 
                        flexDriveMotors[0] = MotorControllerFactory.createSparkFlex(driveFrontLeftPort), 
                        flexTurnMotors[0] = MotorControllerFactory.createSparkFlex(turnFrontLeftPort), 
                        turnEncoders[0] = SensorFactory.createCANCoder(canCoderPortFL), 0, pitchSupplier, rollSupplier);
                    moduleFR = new SwerveModule(swerveConfig, SwerveModule.ModuleType.FR, 
                        flexDriveMotors[1] = MotorControllerFactory.createSparkFlex(driveFrontRightPort), 
                        flexTurnMotors[1] = MotorControllerFactory.createSparkFlex(turnFrontRightPort), 
                        turnEncoders[1] = SensorFactory.createCANCoder(canCoderPortFR), 1, pitchSupplier, rollSupplier);

                    moduleBL = new SwerveModule(swerveConfig, SwerveModule.ModuleType.BL, 
                        flexDriveMotors[2] = MotorControllerFactory.createSparkFlex(driveBackLeftPort), 
                        flexTurnMotors[2] = MotorControllerFactory.createSparkFlex(turnBackLeftPort), 
                        turnEncoders[2] = SensorFactory.createCANCoder(canCoderPortBL), 2, pitchSupplier, rollSupplier);

                    moduleBR = new SwerveModule(swerveConfig, SwerveModule.ModuleType.BR, 
                        flexDriveMotors[3] = MotorControllerFactory.createSparkFlex(driveBackRightPort), 
                        flexTurnMotors[3] = MotorControllerFactory.createSparkFlex(turnBackRightPort),
                        turnEncoders[3] = SensorFactory.createCANCoder(canCoderPortBR), 3, pitchSupplier, rollSupplier);
                    modules = new SwerveModule[] { moduleFL, moduleFR, moduleBL, moduleBR };
                    SparkFlexConfig driveConfig = new SparkFlexConfig();
                    driveConfig.openLoopRampRate(secsPer12Volts);
                    driveConfig.encoder.positionConversionFactor(wheelDiameterMeters * Math.PI / driveGearing);
                    driveConfig.encoder.velocityConversionFactor(wheelDiameterMeters * Math.PI / driveGearing / 60);
                    driveConfig.encoder.uvwAverageDepth(2);
                    driveConfig.encoder.uvwMeasurementPeriod(16);
                    driveConfig.smartCurrentLimit(MotorConfig.NEO.currentLimitAmps);

                    for (SparkFlex driveMotor : flexDriveMotors) {
                        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                    }
                    SparkFlexConfig turnConfig = new SparkFlexConfig();
                    turnConfig.encoder.positionConversionFactor(360/turnGearing);
                    turnConfig.encoder.velocityConversionFactor(360/turnGearing/60);
                    turnConfig.encoder.uvwAverageDepth(2);
                    turnConfig.encoder.uvwMeasurementPeriod(16);

                    //turnConfig.closedLoop.pid(kP, kI, kD).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
                    for (SparkFlex turnMotor : flexTurnMotors) {
                        turnMotor.configure(turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                    }
                    turnPidControllers[0] = flexTurnMotors[0].getClosedLoopController();
                    turnPidControllers[1] = flexTurnMotors[1].getClosedLoopController();
                    turnPidControllers[2] = flexTurnMotors[2].getClosedLoopController();
                    turnPidControllers[3] = flexTurnMotors[3].getClosedLoopController();
                }
                else {
                    moduleFL = new SwerveModule(swerveConfig, SwerveModule.ModuleType.FL, 
                        flexDriveMotors[0] = MotorControllerFactory.createSparkFlex(driveFrontLeftPort), 
                        maxTurnMotors[0] = MotorControllerFactory.createSparkMax(turnFrontLeftPort, MotorConfig.NEO), 
                        turnEncoders[0] = SensorFactory.createCANCoder(canCoderPortFL), 0, pitchSupplier, rollSupplier);
                    moduleFR = new SwerveModule(swerveConfig, SwerveModule.ModuleType.FR, 
                        flexDriveMotors[1] = MotorControllerFactory.createSparkFlex(driveFrontRightPort), 
                        maxTurnMotors[1] = MotorControllerFactory.createSparkMax(turnFrontRightPort, MotorConfig.NEO), 
                        turnEncoders[1] = SensorFactory.createCANCoder(canCoderPortFR), 1, pitchSupplier, rollSupplier);

                    moduleBL = new SwerveModule(swerveConfig, SwerveModule.ModuleType.BL, 
                        flexDriveMotors[2] = MotorControllerFactory.createSparkFlex(driveBackLeftPort), 
                        maxTurnMotors[2] = MotorControllerFactory.createSparkMax(turnBackLeftPort, MotorConfig.NEO), 
                        turnEncoders[2] = SensorFactory.createCANCoder(canCoderPortBL), 2, pitchSupplier, rollSupplier);

                    moduleBR = new SwerveModule(swerveConfig, SwerveModule.ModuleType.BR, 
                        flexDriveMotors[3] = MotorControllerFactory.createSparkFlex(driveBackRightPort), 
                        maxTurnMotors[3] = MotorControllerFactory.createSparkMax(turnBackRightPort, MotorConfig.NEO),
                        turnEncoders[3] = SensorFactory.createCANCoder(canCoderPortBR), 3, pitchSupplier, rollSupplier);
                    modules = new SwerveModule[] { moduleFL, moduleFR, moduleBL, moduleBR };
                    SparkFlexConfig driveConfig = new SparkFlexConfig();
                    driveConfig.openLoopRampRate(secsPer12Volts);
                    driveConfig.encoder.positionConversionFactor(wheelDiameterMeters * Math.PI / driveGearing);
                    driveConfig.encoder.velocityConversionFactor(wheelDiameterMeters * Math.PI / driveGearing / 60);
                    driveConfig.encoder.uvwAverageDepth(2);
                    driveConfig.encoder.uvwMeasurementPeriod(16);
                    driveConfig.smartCurrentLimit(MotorConfig.NEO.currentLimitAmps);

                    for (SparkFlex driveMotor : flexDriveMotors) {
                        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                    }
                    SparkMaxConfig turnConfig = new SparkMaxConfig();
                    turnConfig.encoder.positionConversionFactor(360/turnGearing);
                    turnConfig.encoder.velocityConversionFactor(360/turnGearing/60);
                    turnConfig.encoder.uvwAverageDepth(2);
                    turnConfig.encoder.uvwMeasurementPeriod(16);

                    //turnConfig.closedLoop.pid(kP, kI, kD).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
                    for (SparkMax turnMotor : maxTurnMotors) {
                        turnMotor.configure(turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                    }
                    turnPidControllers[0] = maxTurnMotors[0].getClosedLoopController();
                    turnPidControllers[1] = maxTurnMotors[1].getClosedLoopController();
                    turnPidControllers[2] = maxTurnMotors[2].getClosedLoopController();
                    turnPidControllers[3] = maxTurnMotors[3].getClosedLoopController();
                }
            }
            else {
                if (useFlexTurn) {
                    moduleFL = new SwerveModule(swerveConfig, SwerveModule.ModuleType.FL, 
                        maxDriveMotors[0] = MotorControllerFactory.createSparkMax(driveFrontLeftPort, MotorConfig.NEO), 
                        flexTurnMotors[0] = MotorControllerFactory.createSparkFlex(turnFrontLeftPort), 
                        turnEncoders[0] = SensorFactory.createCANCoder(canCoderPortFL), 0, pitchSupplier, rollSupplier);
                    moduleFR = new SwerveModule(swerveConfig, SwerveModule.ModuleType.FR, 
                        maxDriveMotors[1] = MotorControllerFactory.createSparkMax(driveFrontRightPort, MotorConfig.NEO), 
                        flexTurnMotors[1] = MotorControllerFactory.createSparkFlex(turnFrontRightPort), 
                        turnEncoders[1] = SensorFactory.createCANCoder(canCoderPortFR), 1, pitchSupplier, rollSupplier);

                    moduleBL = new SwerveModule(swerveConfig, SwerveModule.ModuleType.BL, 
                        maxDriveMotors[2] = MotorControllerFactory.createSparkMax(driveBackLeftPort, MotorConfig.NEO), 
                        flexTurnMotors[2] = MotorControllerFactory.createSparkFlex(turnBackLeftPort), 
                        turnEncoders[2] = SensorFactory.createCANCoder(canCoderPortBL), 2, pitchSupplier, rollSupplier);

                    moduleBR = new SwerveModule(swerveConfig, SwerveModule.ModuleType.BR, 
                        maxDriveMotors[3] = MotorControllerFactory.createSparkMax(driveBackRightPort, MotorConfig.NEO), 
                        flexTurnMotors[3] = MotorControllerFactory.createSparkFlex(turnBackRightPort),
                        turnEncoders[3] = SensorFactory.createCANCoder(canCoderPortBR), 3, pitchSupplier, rollSupplier);
                    modules = new SwerveModule[] { moduleFL, moduleFR, moduleBL, moduleBR };
                    SparkMaxConfig driveConfig = new SparkMaxConfig();
                    driveConfig.openLoopRampRate(secsPer12Volts);
                    driveConfig.encoder.positionConversionFactor(wheelDiameterMeters * Math.PI / driveGearing);
                    driveConfig.encoder.velocityConversionFactor(wheelDiameterMeters * Math.PI / driveGearing / 60);
                    driveConfig.encoder.uvwAverageDepth(2);
                    driveConfig.encoder.uvwMeasurementPeriod(16);
                    driveConfig.smartCurrentLimit(MotorConfig.NEO.currentLimitAmps);

                    for (SparkMax driveMotor : maxDriveMotors) {
                        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                    }
                    SparkFlexConfig turnConfig = new SparkFlexConfig();
                    turnConfig.encoder.positionConversionFactor(360/turnGearing);
                    turnConfig.encoder.velocityConversionFactor(360/turnGearing/60);
                    turnConfig.encoder.uvwAverageDepth(2);
                    turnConfig.encoder.uvwMeasurementPeriod(16);

                    //turnConfig.closedLoop.pid(kP, kI, kD).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
                    for (SparkFlex turnMotor : flexTurnMotors) {
                        turnMotor.configure(turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                    }
                    turnPidControllers[0] = flexTurnMotors[0].getClosedLoopController();
                    turnPidControllers[1] = flexTurnMotors[1].getClosedLoopController();
                    turnPidControllers[2] = flexTurnMotors[2].getClosedLoopController();
                    turnPidControllers[3] = flexTurnMotors[3].getClosedLoopController();
                }
                else {
                    moduleFL = new SwerveModule(swerveConfig, SwerveModule.ModuleType.FL, 
                        maxDriveMotors[0] = MotorControllerFactory.createSparkMax(driveFrontLeftPort, MotorConfig.NEO), 
                        maxTurnMotors[0] = MotorControllerFactory.createSparkMax(turnFrontLeftPort, MotorConfig.NEO), 
                        turnEncoders[0] = SensorFactory.createCANCoder(canCoderPortFL), 0, pitchSupplier, rollSupplier);
                    moduleFR = new SwerveModule(swerveConfig, SwerveModule.ModuleType.FR, 
                        maxDriveMotors[1] = MotorControllerFactory.createSparkMax(driveFrontRightPort, MotorConfig.NEO), 
                        maxTurnMotors[1] = MotorControllerFactory.createSparkMax(turnFrontRightPort, MotorConfig.NEO), 
                        turnEncoders[1] = SensorFactory.createCANCoder(canCoderPortFR), 1, pitchSupplier, rollSupplier);

                    moduleBL = new SwerveModule(swerveConfig, SwerveModule.ModuleType.BL, 
                        maxDriveMotors[2] = MotorControllerFactory.createSparkMax(driveBackLeftPort, MotorConfig.NEO), 
                        maxTurnMotors[2] = MotorControllerFactory.createSparkMax(turnBackLeftPort, MotorConfig.NEO), 
                        turnEncoders[2] = SensorFactory.createCANCoder(canCoderPortBL), 2, pitchSupplier, rollSupplier);

                    moduleBR = new SwerveModule(swerveConfig, SwerveModule.ModuleType.BR, 
                        maxDriveMotors[3] = MotorControllerFactory.createSparkMax(driveBackRightPort, MotorConfig.NEO), 
                        maxTurnMotors[3] = MotorControllerFactory.createSparkMax(turnBackRightPort, MotorConfig.NEO),
                        turnEncoders[3] = SensorFactory.createCANCoder(canCoderPortBR), 3, pitchSupplier, rollSupplier);
                    modules = new SwerveModule[] { moduleFL, moduleFR, moduleBL, moduleBR };
                    SparkMaxConfig driveConfig = new SparkMaxConfig();
                    driveConfig.openLoopRampRate(secsPer12Volts);
                    driveConfig.encoder.positionConversionFactor(wheelDiameterMeters * Math.PI / driveGearing);
                    driveConfig.encoder.velocityConversionFactor(wheelDiameterMeters * Math.PI / driveGearing / 60);
                    driveConfig.encoder.uvwAverageDepth(2);
                    driveConfig.encoder.uvwMeasurementPeriod(16);
                    driveConfig.smartCurrentLimit(MotorConfig.NEO.currentLimitAmps);

                    for (SparkMax driveMotor : maxDriveMotors) {
                        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                    }
                    SparkMaxConfig turnConfig = new SparkMaxConfig();
                    turnConfig.encoder.positionConversionFactor(360/turnGearing);
                    turnConfig.encoder.velocityConversionFactor(360/turnGearing/60);
                    turnConfig.encoder.uvwAverageDepth(2);
                    turnConfig.encoder.uvwMeasurementPeriod(16);

                    //turnConfig.closedLoop.pid(kP, kI, kD).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
                    for (SparkMax turnMotor : maxTurnMotors) {
                        turnMotor.configure(turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                    }
                    turnPidControllers[0] = maxTurnMotors[0].getClosedLoopController();
                    turnPidControllers[1] = maxTurnMotors[1].getClosedLoopController();
                    turnPidControllers[2] = maxTurnMotors[2].getClosedLoopController();
                    turnPidControllers[3] = maxTurnMotors[3].getClosedLoopController();
                }
            }
            if (RobotBase.isSimulation()) {
                moduleSims = new SwerveModuleSim[] {
                    moduleFL.createSim(), moduleFR.createSim(), moduleBL.createSim(), moduleBR.createSim()
                };
                gyroYawSim = new SimDeviceSim("navX-Sensor[0]").getDouble("Yaw");
            }
            
            SmartDashboard.putData("Module FL",moduleFL);
            SmartDashboard.putData("Module FR",moduleFR);
            SmartDashboard.putData("Module BL",moduleBL);
            SmartDashboard.putData("Module BR",moduleBR);
            
            for (CANcoder coder : turnEncoders) {
                coder.getAbsolutePosition().setUpdateFrequency(500);
                coder.getPosition().setUpdateFrequency(500);
                coder.getVelocity().setUpdateFrequency(500);
            }

            accelX = gyro.getWorldLinearAccelX(); // Acceleration along the X-axis
            accelY = gyro.getWorldLinearAccelY(); // Acceleration along the Y-axis
            accelXY = Math.sqrt(gyro.getWorldLinearAccelX() * gyro.getWorldLinearAccelX() + gyro.getWorldLinearAccelY() * gyro.getWorldLinearAccelY());


            // Must call this method for SysId to run
            if (setupSysId) {
                sysIdSetup();
            }
        }

        odometry = new SwerveDriveOdometry(kinematics,
        Rotation2d.fromDegrees(getHeading()), getModulePositions(),
        new Pose2d());
        poseEstimator = new SwerveDrivePoseEstimator(
                getKinematics(),
                Rotation2d.fromDegrees(getHeading()),
                getModulePositions(),
                new Pose2d());
        
        SmartDashboard.putData(this);

    }


    public boolean isAtAngle(double desiredAngleDeg, double toleranceDeg){
        for (SwerveModule module : modules) { 
            if (!(Math.abs(MathUtil.inputModulus(module.getModuleAngle() - desiredAngleDeg, -90, 90)) < toleranceDeg)) 
                return false;
        }
        return true;
    }

    @Override
    public void simulationPeriodic() {
        for (var moduleSim : moduleSims) {
            moduleSim.update();
        }
        SwerveModuleState[] measuredStates =
            new SwerveModuleState[] {
                moduleFL.getCurrentState(), moduleFR.getCurrentState(), moduleBL.getCurrentState(), moduleBR.getCurrentState()
            };
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(measuredStates);

        double dtSecs = simTimer.get();
        simTimer.restart();

        Pose2d simPose = field.getRobotPose();
        simPose = simPose.exp(
                new Twist2d(
                    speeds.vxMetersPerSecond * dtSecs,
                    speeds.vyMetersPerSecond * dtSecs,
                    speeds.omegaRadiansPerSecond * dtSecs));
        double newAngleDeg = simPose.getRotation().getDegrees();
        // Subtract the offset computed the last time setPose() was called because odometry.update() adds it back.
        newAngleDeg -= simGyroOffset.getDegrees();
        newAngleDeg *= (isGyroReversed ? -1.0 : 1.0);
        gyroYawSim.set(newAngleDeg);
    }

    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction, int
    // frontorback) {
    // switch(frontorback) {
    // case 0:
    // return frontOnlyRoutine.quasistatic(direction);
    // case 1:
    // return backOnlyRoutine.quasistatic(direction);
    // case 2:
    // return allWheelsRoutine.quasistatic(direction);
    // }
    // return new PrintCommand("Invalid Command");
    // }

    /**
     * Sets swerveModules IdleMode both turn and drive
     * @param brake boolean for braking, if false then coast
     */
    public void setDrivingIdleMode(boolean brake) {
        IdleMode mode;
        if (brake) {
            mode = IdleMode.kBrake;
        }
        else {
            mode = IdleMode.kCoast;
        }
        if (useFlexDrive) {
            for (SparkFlex driveMotor : flexDriveMotors) {
                SparkFlexConfig config = new SparkFlexConfig();
                config.idleMode(mode);
                driveMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);     
            }
        }
        else {
            for (SparkMax driveMotor : maxDriveMotors) {
                SparkMaxConfig config = new SparkMaxConfig();
                config.idleMode(mode);
                driveMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);     
            }
        }
        if (useFlexTurn) {
            for (SparkFlex turnMotor : flexTurnMotors) {
                SparkFlexConfig config = new SparkFlexConfig();
                config.idleMode(mode);
                turnMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);      
            }
        }
        else {
            for (SparkMax turnMotor : maxTurnMotors) {
                SparkMaxConfig config = new SparkMaxConfig();
                config.idleMode(mode);
                turnMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);      
            }
        }
        
    }

    @Override
    public void periodic() {
        detectCollision(); //This does nothing
        PathPlannerLogging.logCurrentPose(getPose());
        
        for (SwerveModule module : modules) {
          // module.turnPeriodic(); For testing
          module.periodic();
        }

        // field.setRobotPose(odometry.getPoseMeters());

        

        // odometry.update(gyro.getRotation2d(), getModulePositions());

        // poseEstimator.update(gyro.getRotation2d(), getModulePositions());
        
        //odometry.update(Rotation2d.fromDegrees(getHeading()), getModulePositions());

        // updateMT2PoseEstimator();

        // double currSetX =
        // SmartDashboard.getNumber("Pose Estimator set x (m)", lastSetX);
        // double currSetY =
        // SmartDashboard.getNumber("Pose Estimator set y (m)", lastSetY);
        // double currSetTheta = SmartDashboard
        // .getNumber("Pose Estimator set rotation (deg)", lastSetTheta);

        // if (lastSetX != currSetX || lastSetY != currSetY
        // || lastSetTheta != currSetTheta) {
        // setPose(new Pose2d(currSetX, currSetY,
        // Rotation2d.fromDegrees(currSetTheta)));
        // }

        // setPose(new Pose2d(getPose().getTranslation().getX(),
        // getPose().getTranslation().getY(),
        // Rotation2d.fromDegrees(getHeading())));


        // SmartDashboard.putNumber("X position with limelight", getPoseWithLimelight().getX());
        // SmartDashboard.putNumber("Y position with limelight", getPoseWithLimelight().getY());
        SmartDashboard.putNumber("X position with gyro", getPose().getX());
        SmartDashboard.putNumber("Y position with gyro", getPose().getY());
        SmartDashboard.putBoolean("setupSysId", setupSysId);
        
        //For finding acceleration of drivetrain for collision detector
        SmartDashboard.putNumber("Accel X", accelX);
        SmartDashboard.putNumber("Accel Y", accelY);
        SmartDashboard.putNumber("2D Acceleration ", accelXY);

        // // // SmartDashboard.putNumber("Pitch", gyro.getPitch());
        // // // SmartDashboard.putNumber("Roll", gyro.getRoll());
        // SmartDashboard.putNumber("Raw gyro angle", gyro.getAngle());
        // SmartDashboard.putNumber("Robot Heading", getHeading());
        // // // SmartDashboard.putNumber("AdjRoll", gyro.getPitch() - initPitch);
        // // // SmartDashboard.putNumber("AdjPitch", gyro.getRoll() - initRoll);
        // SmartDashboard.putBoolean("Field Oriented", fieldOriented);
        // SmartDashboard.putNumber("Gyro Compass Heading", gyro.getCompassHeading());
        // SmartDashboard.putNumber("Compass Offset", compassOffset);
        // SmartDashboard.putBoolean("Current Magnetic Field Disturbance", gyro.isMagneticDisturbance());
        SmartDashboard.putNumber("front left encoder", moduleFL.getModuleAngle());
        SmartDashboard.putNumber("front right encoder", moduleFR.getModuleAngle());
        SmartDashboard.putNumber("back left encoder", moduleBL.getModuleAngle());
        SmartDashboard.putNumber("back right encoder", moduleBR.getModuleAngle());
        userPeriodic();
    }

    public void userPeriodic() {
        //Override this
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        for (SwerveModule module : modules)
            SendableRegistry.addChild(this, module);
        
        builder.addBooleanProperty("Magnetic Field Disturbance",
        gyro::isMagneticDisturbance, null);
        builder.addBooleanProperty("Gyro Calibrating", gyro::isCalibrating, null);
        builder.addBooleanProperty("Field Oriented", () -> fieldOriented,
        fieldOriented -> this.fieldOriented = fieldOriented);
        builder.addDoubleProperty("Pose Estimator X", () -> getPose().getX(),
                null);
        builder.addDoubleProperty("Pose Estimator Y", () -> getPose().getY(),
                null);
        builder.addDoubleProperty("Pose Estimator Theta",
                () ->
        getPose().getRotation().getDegrees(), null);
        builder.addDoubleProperty("Robot Heading", () -> getHeading(), null);
        builder.addDoubleProperty("Raw Gyro Angle", gyro::getAngle, null);
        builder.addDoubleProperty("Pitch", gyro::getPitch, null);
        builder.addDoubleProperty("Roll", gyro::getRoll, null);
        builder.addDoubleProperty("Field Offset", () -> fieldOffset, fieldOffset ->
        this.fieldOffset = fieldOffset);
        builder.addDoubleProperty("FL Turn Encoder (Deg)",
                () -> moduleFL.getModuleAngle(), null);
        builder.addDoubleProperty("FR Turn Encoder (Deg)",
                () -> moduleFR.getModuleAngle(), null);
        builder.addDoubleProperty("BL Turn Encoder (Deg)",
                () -> moduleBL.getModuleAngle(), null);
        builder.addDoubleProperty("BR Turn Encoder (Deg)",
                () -> moduleBR.getModuleAngle(), null);
    }


    // #region Drive Methods

    /**
     * Drives the robot using the given x, y, and rotation speed
     *
     * @param forward  The desired forward speed, in m/s. Forward is positive.
     * @param strafe   The desired strafe speed, in m/s. Left is positive.
     * @param rotation The desired rotation speed, in rad/s. Counter clockwise is
     *                 positive
     */
    public void setExtraSpeedMult(double set) {
        extraSpeedMult=set;
    }
    
    /**
     * Calculates and implements the required SwerveStates for all 4 modules to get the wanted outcome
     * @param forward The desired forward speed, in m/s. Forward is positive.
     * @param strafe The desired strafe speed, in m/s. Left is positive.
     * @param rotation The desired rotation speed, in rad/s. Counter clockwise is positive.
     */
    public void drive(double forward, double strafe, double rotation) {
        drive(getSwerveStates(forward, strafe, rotation));
    }
    
    /**
     * Implements the provided SwerveStates for all 4 modules to get the wanted outcome
     * @param moduleStates SwerveModuleState[]
     */
    public void drive(SwerveModuleState[] moduleStates) {
        //Max speed override
        double max = maxSpeed;
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, max);
        for (int i = 0; i < 4; i++) {
            // SmartDashboard.putNumber("moduleIn" + Integer.toString(i), moduleStates[i].angle.getDegrees());
            moduleStates[i].optimize(Rotation2d.fromDegrees(modules[i].getModuleAngle()));
            // SmartDashboard.putNumber("moduleOT" + Integer.toString(i), moduleStates[i].angle.getDegrees());
            modules[i].move(moduleStates[i].speedMetersPerSecond, moduleStates[i].angle.getDegrees());
        }
    }
    
    /**
     * Configures PathPlanner AutoBuilder
     */
    public void AutoBuilder() {
        RobotConfig config = robotConfig;
        AutoBuilder.configure(
                //Supplier<Pose2d> poseSupplier,
                this::getPose, // Robot pose supplier
                //Consumer<Pose2d> resetPose,
                this::setPoseWithLimelight, // Method to reset odometry (will be called if your auto has a starting pose)
                //Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                //BiConsumer<ChassisSpeeds,DriveFeedforwards> output,
                (speeds, feedforwards) -> drive(kinematics.toSwerveModuleStates(speeds)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                //PathFollowingController controller,
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(4
                        , ppKiDrive, ppKdDrive), // Translation PID constants
                        new PIDConstants(1, ppKiTurn, ppKdTurn)
                ),
                //RobotConfig robotConfig,
                config, // The robot configuration
                //BooleanSupplier shouldFlipPath,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                //Subsystem... driveRequirements
                this // Reference to this subsystem to set requirements
            );
        }

//----------------------------------------------------------

   public void autoCancelDtCommand() {
       if(!(getDefaultCommand() instanceof SimpleTeleopDrive) || DriverStation.isAutonomous()) return;

        // Use hasDriverInput to get around acceleration limiting on slowdown
        if (((SimpleTeleopDrive) getDefaultCommand()).hasDriverInput()) {
            Command currentDtCommand = getCurrentCommand();
            if (currentDtCommand != getDefaultCommand() && !(currentDtCommand instanceof SimpleRotateToFieldRelativeAngle)
                    && currentDtCommand != null) {
                currentDtCommand.cancel();
            }
        }
    }

    public void stop() {
        for (SwerveModule module : modules)
            module.move(0, 0);
    }

    public boolean isStopped() {
        return Math.abs(getSpeeds().vxMetersPerSecond) < 0.1 &&
                Math.abs(getSpeeds().vyMetersPerSecond) < 0.1 &&
                Math.abs(getSpeeds().omegaRadiansPerSecond) < 0.1;
    }

    /**
     * Constructs and returns a ChassisSpeeds objects using forward, strafe, and
     * rotation values.
     *
     * @param forward  The desired forward speed, in m/s. Forward is positive.
     * @param strafe   The desired strafe speed, in m/s. Left is positive.
     * @param rotation The desired rotation speed, in rad/s. Counter clockwise is
     *                 positive.
     * @return A ChassisSpeeds object.
     */
    private ChassisSpeeds getChassisSpeeds(double forward, double strafe, double rotation) {
        ChassisSpeeds speeds;
        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation,
                    Rotation2d.fromDegrees(getHeading()));
        } else {
            speeds = new ChassisSpeeds(forward, strafe, rotation);
        }
        return speeds;
    }

    /**
     * Constructs and returns four SwerveModuleState objects, one for each side,
     * using forward, strafe, and rotation values.
     *
     * @param forward  The desired forward speed, in m/s. Forward is positive.
     * @param strafe   The desired strafe speed, in m/s. Left is positive.
     * @param rotation The desired rotation speed, in rad/s. Counter clockwise is
     *                 positive.
     * @return A SwerveModuleState array, one for each side of the drivetrain (FL,
     *         FR, etc.).
     */
    private SwerveModuleState[] getSwerveStates(double forward, double strafe, double rotation) {
        return kinematics.toSwerveModuleStates(getChassisSpeeds(forward, -strafe, rotation));
    }

    public SwerveModuleState[] getSwerveStates(ChassisSpeeds speeds) {
        return kinematics.toSwerveModuleStates(speeds);
    }

    // #endregion

    // #region Getters and Setters

    /**
     * @return the heading in degrees, normalized to the range -180 to 180
     */
    public double getHeading() {
        double x = gyro.getAngle();
        if (fieldOriented)
            x -= fieldOffset;
        return Math.IEEEremainder(x * (isGyroReversed ? -1.0 : 1.0), 360);
    }

    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules).map(SwerveModule::getCurrentPosition).toArray(SwerveModulePosition[]::new);
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    /**
     * Gets pose from {@link #poseEstimator}
     * @return Pose2D
     */
    public Pose2d getPose() {
        // return odometry.getPoseMeters();
        return poseEstimator.getEstimatedPosition();
    }
    
    public void setPose(Pose2d initialPose) {
        Rotation2d gyroRotation = gyro.getRotation2d();
        poseEstimator.resetPosition(gyroRotation, getModulePositions(), initialPose);
        simGyroOffset = initialPose.getRotation().minus(gyroRotation);
        odometry.resetPosition(gyroRotation, getModulePositions(), initialPose);
    }

    //This method will set the pose using limelight if it sees a tag and if not it is supposed to run like setPose()
    public void setPoseWithLimelight(Pose2d backupPose){
        DriverStation.reportError("Need to override setPoseWithLimelight and make the method setPose", true);
    }

    /**
     * Detects if the robot has experienced a collision based on acceleration thresholds.
     * @return true if a 2D acceleration greater than the {@link #COLLISION_ACCELERATION_THRESHOLD} false otherwise.
     */
    public boolean detectCollision(){ //We can implement this method into updatePoseWithLimelight so that if there is a collision it stops using odometry
        accelX = gyro.getWorldLinearAccelX(); // Acceleration along the X-axis
        accelY = gyro.getWorldLinearAccelY(); // Acceleration along the Y-axis
        accelXY = Math.sqrt(accelX * accelX + accelY * accelY); // 2D Acceleration
        return accelXY > COLLISION_ACCELERATION_THRESHOLD; // return true if collision detected
    }

    /**
     * @deprecated Use {@link #resetFieldOrientation()} instead
     */
    @Deprecated
    public void resetHeading() {
        gyro.reset();
        
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    /**
     * true stands for fieldOriented, false stands for robotOriented
     * @return boolean
     */
    public boolean getFieldOriented() {
        return fieldOriented;
    }

    /**
     * True sets fieldOriented, false sets robotOriented
     * @param fieldOriented boolean
     */
    public void setFieldOriented(boolean fieldOriented) {
        this.fieldOriented = fieldOriented;
    }

    /**
     * Sets the current direction the robot is facing is to be 0
     */
    public void resetFieldOrientation() {
        fieldOffset = gyro.getAngle();
    }

    /**
     * Sets the current direction the robot is facing is to be 180
     */
    public void resetFieldOrientationBackwards() {
        fieldOffset = 180 + gyro.getAngle();
    }

    /**
     * Sets the current direction the robot is facing is plus @param angle to be 0
     * @param angle in degrees
     */
    public void resetFieldOrientationWithAngle(double angle) {
        fieldOffset = angle + gyro.getAngle();
    }

    public void resetPoseEstimator() {
        poseEstimator.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d());
        gyro.reset();
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(Arrays.stream(modules).map(SwerveModule::getCurrentState)
                .toArray(SwerveModuleState[]::new));
    }

    public void setMode(Mode mode) {
        for (SwerveModule module : modules){
            switch (mode) {
                case coast:
                    module.coast();
                    break;
                case brake:
                    module.brake();
                    break;
                case toggle:
                    module.toggleMode();
                    break;
            }
        }
    }
    /**
     * @deprecated Use {@link #setMode(Mode)} instead
     * Changes between IdleModes
     */
    @Deprecated
    public void toggleMode() {
        for (SwerveModule module : modules)
            module.toggleMode();
    }
    /** 
     * @deprecated Use {@link #setMode(Mode)} instead
     */
    @Deprecated
    public void brake() {
        for (SwerveModule module : modules)
            module.brake();
    }
    /**
     * @deprecated Use {@link #setMode(Mode)} instead
     */
    @Deprecated
    public void coast() {
        for (SwerveModule module : modules)
            module.coast();
    }



    // #region SysId Code

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage[] m_appliedVoltage = new MutVoltage[8];

    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutDistance[] m_distance = new MutDistance[4];
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutLinearVelocity[] m_velocity = new MutLinearVelocity[4];
    // edu.wpi.first.math.util.Units.Rotations beans;
    private final MutAngle[] m_revs = new MutAngle[4];
    private final MutAngularVelocity[] m_revs_vel = new MutAngularVelocity[4];

    private enum SysIdTest {
        FRONT_DRIVE,
        BACK_DRIVE,
        ALL_DRIVE,
        // FLBR_TURN,
        // FRBL_TURN,
        // ALL_TURN
        FL_ROT,
        FR_ROT,
        BL_ROT,
        BR_ROT
    }

    private SendableChooser<SysIdTest> sysIdChooser = new SendableChooser<>();

    // ROUTINES FOR SYSID
    // private SysIdRoutine.Config defaultSysIdConfig = new
    // SysIdRoutine.Config(Volts.of(.1).per(Seconds.of(.1)), Volts.of(.6),
    // Seconds.of(5));
    private SysIdRoutine.Config defaultSysIdConfig = new SysIdRoutine.Config(Volts.of(1).per(Seconds),
            Volts.of(2.891), Seconds.of(10));

    // DRIVE
    private void motorLogShort_drive(SysIdRoutineLog log, int id) {
        if (useFlexDrive) {
            String name = new String[] { "fl", "fr", "bl", "br" }[id];
        log.motor(name)
                .voltage(m_appliedVoltage[id].mut_replace(
                        flexDriveMotors[id].getBusVoltage() * flexDriveMotors[id].getAppliedOutput(), Volts))
                .linearPosition(
                        m_distance[id].mut_replace(flexDriveMotors[id].getEncoder().getPosition(), Meters))
                .linearVelocity(m_velocity[id].mut_replace(flexDriveMotors[id].getEncoder().getVelocity(),
                        MetersPerSecond));
        }
        else {
            String name = new String[] { "fl", "fr", "bl", "br" }[id];
        log.motor(name)
                .voltage(m_appliedVoltage[id].mut_replace(
                        maxDriveMotors[id].getBusVoltage() * maxDriveMotors[id].getAppliedOutput(), Volts))
                .linearPosition(
                        m_distance[id].mut_replace(maxDriveMotors[id].getEncoder().getPosition(), Meters))
                .linearVelocity(m_velocity[id].mut_replace(maxDriveMotors[id].getEncoder().getVelocity(),
                        MetersPerSecond));
        }
    }

    // Create a new SysId routine for characterizing the drive.
    private SysIdRoutine frontOnlyDriveRoutine = new SysIdRoutine(
            defaultSysIdConfig,
            new SysIdRoutine.Mechanism(
                    // Tell SysId how to give the driving voltage to the motors.
                    (Voltage volts) -> {
                        if (useFlexDrive) {
                            flexDriveMotors[0].setVoltage(volts.in(Volts));
                            flexDriveMotors[1].setVoltage(volts.in(Volts));
                        }
                        else { 
                            maxDriveMotors[0].setVoltage(volts.in(Volts));
                            maxDriveMotors[1].setVoltage(volts.in(Volts));
                        }
                        modules[2].coast();
                        modules[3].coast();
                    },
                    log -> {// FRONT
                        motorLogShort_drive(log, 0);// fl named automatically
                        motorLogShort_drive(log, 1);// fr
                    },
                    this));

    private SysIdRoutine backOnlyDriveRoutine = new SysIdRoutine(
            defaultSysIdConfig,
            new SysIdRoutine.Mechanism(
                    (Voltage volts) -> {
                        modules[0].coast();
                        modules[1].coast();
                        modules[2].brake();
                        modules[3].brake();
                        if (useFlexDrive) {
                            flexDriveMotors[2].setVoltage(volts.in(Volts));
                            flexDriveMotors[3].setVoltage(volts.in(Volts));
                        }
                        else {
                            maxDriveMotors[2].setVoltage(volts.in(Volts));
                            maxDriveMotors[3].setVoltage(volts.in(Volts));
                        }
                    },
                    log -> {// BACK
                        motorLogShort_drive(log, 2);// bl
                        motorLogShort_drive(log, 3);// br
                    },
                    this));

    private SysIdRoutine allWheelsDriveRoutine = new SysIdRoutine(
            defaultSysIdConfig,
            new SysIdRoutine.Mechanism(
                    (Voltage volts) -> {
                        if (useFlexDrive) {
                            for (SparkFlex dm : flexDriveMotors) {
                                dm.setVoltage(volts.in(Volts));
                            }
                        }
                        else {
                            for (SparkMax dm : maxDriveMotors) {
                                dm.setVoltage(volts.in(Volts));
                            }
                        }
                    },
                    log -> {
                        motorLogShort_drive(log, 0);// fl named automatically
                        motorLogShort_drive(log, 1);// fr
                        motorLogShort_drive(log, 2);// bl
                        motorLogShort_drive(log, 3);// br
                    },
                    this));

    private SysIdRoutine sysidroutshort_turn(int id, String logname) {
        double busVoltage;
        double appliedOutput;
        if (useFlexTurn) {
            busVoltage = flexTurnMotors[id].getBusVoltage();
            appliedOutput = flexTurnMotors[id].getAppliedOutput();
        }
        else {
            busVoltage = maxTurnMotors[id].getBusVoltage();
            appliedOutput = maxTurnMotors[id].getAppliedOutput();
        }
        return new SysIdRoutine(
                defaultSysIdConfig,
                // new SysIdRoutine.Config(Volts.of(.1).per(Seconds.of(.1)), Volts.of(.6),
                // Seconds.of(3)),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {if (useFlexTurn) {flexTurnMotors[id].setVoltage(volts.in(Volts));} else {maxTurnMotors[id].setVoltage(volts.in(Volts));}},
                        log -> log.motor(logname + "_turn")
                                .voltage(m_appliedVoltage[id + 4].mut_replace(
                                        // ^because drivemotors take up the first 4 slots of the unit holders
                                        busVoltage * appliedOutput, Volts))
                                .angularPosition(
                                        m_revs[id].mut_replace(turnEncoders[id].getPosition().getValue()))
                                .angularVelocity(m_revs_vel[id].mut_replace(
                                        turnEncoders[id].getVelocity().getValueAsDouble(), RotationsPerSecond)),
                        this));
    }

    // as always, fl/fr/bl/br
    private SysIdRoutine[] rotateRoutine = new SysIdRoutine[] {
            sysidroutshort_turn(0, "fl"), // woaw, readable code???
            sysidroutshort_turn(1, "fr"),
            sysidroutshort_turn(2, "bl"),
            sysidroutshort_turn(3, "br")
    };

    //TODO: migrate to elastic
    //private ShuffleboardTab sysIdTab = Shuffleboard.getTab("Drivetrain SysID");

    // void sysidtabshorthand(String name, SysIdRoutine.Direction dir, int width,
    // int height){
    // sysIdTab.add(name, dir).withSize(width, height);
    // }
    void sysidtabshorthand_qsi(String name, SysIdRoutine.Direction dir) {
        //TODO: migrate to elastic
        //sysIdTab.add(name, sysIdQuasistatic(dir)).withSize(2, 1);
    }

    // void sysidtabshorthand_dyn(String name, SysIdRoutine.Direction dir) {
    //     sysIdTab.add(name, sysIdDynamic(dir)).withSize(2, 1);
    // }

    private void sysIdSetup() {
        // SysId Setup
        {
            Supplier<SequentialCommandGroup> stopNwait = () -> new SequentialCommandGroup(
                    new InstantCommand(this::stop), new WaitCommand(2));

            // sysidtabshorthand_qsi("Quasistatic Forward", SysIdRoutine.Direction.kForward);
            // sysidtabshorthand_qsi("Quasistatic Backward", SysIdRoutine.Direction.kReverse);
            // sysidtabshorthand_dyn("Dynamic Forward", SysIdRoutine.Direction.kForward);
            // sysidtabshorthand_dyn("Dynamic Backward", SysIdRoutine.Direction.kReverse);

            //TODO: migrate to elastic
            //sysIdTab
            //.add(sysIdChooser)
            //.withSize(2, 1);

            sysIdChooser.addOption("Front Only Drive", SysIdTest.FRONT_DRIVE);
            sysIdChooser.addOption("Back Only Drive", SysIdTest.BACK_DRIVE);
            sysIdChooser.addOption("All Drive", SysIdTest.ALL_DRIVE);
            // sysIdChooser.addOption("fl-br Turn", SysIdTest.FLBR_TURN);
            // sysIdChooser.addOption("fr-bl Turn", SysIdTest.FRBL_TURN);
            // sysIdChooser.addOption("All Turn", SysIdTest.ALL_TURN);
            sysIdChooser.addOption("FL Rotate", SysIdTest.FL_ROT);
            sysIdChooser.addOption("FR Rotate", SysIdTest.FR_ROT);
            sysIdChooser.addOption("BL Rotate", SysIdTest.BL_ROT);
            sysIdChooser.addOption("BR Rotate", SysIdTest.BR_ROT);

            //TODO: migrate to elastic
            //sysIdTab.add("ALL THE SYSID TESTS", allTheSYSID())// is this legal??
                    //.withSize(2, 1);
            
            // sysIdTab.add("Dynamic Backward", sysIdDynamic(SysIdRoutine.Direction.kReverse)).withSize(2, 1);
            // sysIdTab.add("Dynamic Forward", sysIdDynamic(SysIdRoutine.Direction.kForward)).withSize(2, 1);
            // SmartDashboard.putData("Quackson Backward", sysIdQuasistatic(SysIdRoutine.Direction.kReverse));//.withSize(2, 1);
            // SmartDashboard.putData("Quackson Forward", sysIdQuasistatic(SysIdRoutine.Direction.kForward));//.withSize(2, 1);

            // SmartDashboard.putData("Dyanmic forward", sysIdDynamic(SysIdRoutine.Direction.kForward));//.withSize(2, 1);
            // SmartDashboard.putData("Dyanmic backward", sysIdDynamic(SysIdRoutine.Direction.kReverse));//.withSize(2, 1);
            //sysIdTab.add(this);

            for (int i = 0; i < 8; i++) {// first four are drive, next 4 are turn motors
                m_appliedVoltage[i] = Volt.mutable(0);
            }
            for (int i = 0; i < 4; i++) {
                m_distance[i] = Meter.mutable(0);
                m_velocity[i] = MetersPerSecond.mutable(0);

                m_revs[i] = Rotation.mutable(0);
                m_revs_vel[i] = RotationsPerSecond.mutable(0);
            }

            // SmartDashboard.putNumber("Desired Angle", 0);

            // SmartDashboard.putNumber("kS", 0);
        }
    }

    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction, int
    // frontorback) {
    // switch(frontorback) {
    // case 0:
    // return frontOnlyRoutine.quasistatic(direction);
    // case 1:
    // return backOnlyRoutine.quasistatic(direction);
    // case 2:
    // return allWheelsRoutine.quasistatic(direction);
    // }
    // return new PrintCommand("Invalid Command");
    // }

    private SysIdTest selector() {
        //SysIdTest test = sysIdChooser.getSelected();
        SysIdTest test = SysIdTest.FRONT_DRIVE;
        System.out.println("Test Selected: " + test);
        return test;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return new SelectCommand<>(
                Map.ofEntries(
                        // DRIVE
                        Map.entry(SysIdTest.FRONT_DRIVE, new ParallelCommandGroup(
                                direction == SysIdRoutine.Direction.kForward
                                        ? new PrintCommand("Running front only quasistatic forward")
                                        : new PrintCommand("Running front only quasistatic backward"),
                                frontOnlyDriveRoutine.quasistatic(direction))),
                        Map.entry(SysIdTest.BACK_DRIVE, new ParallelCommandGroup(
                                direction == SysIdRoutine.Direction.kForward
                                        ? new PrintCommand("Running back only quasistatic forward")
                                        : new PrintCommand("Running back only quasistatic backward"),
                                backOnlyDriveRoutine.quasistatic(direction))),
                        Map.entry(SysIdTest.ALL_DRIVE, new ParallelCommandGroup(
                                direction == SysIdRoutine.Direction.kForward
                                        ? new PrintCommand("Running all drive quasistatic forward")
                                        : new PrintCommand("Running all drive quasistatic backward"),
                                allWheelsDriveRoutine.quasistatic(direction))),
                        // ROTATE
                        Map.entry(SysIdTest.FL_ROT, new ParallelCommandGroup(
                                direction == SysIdRoutine.Direction.kForward
                                        ? new PrintCommand("Running FL rotate quasistatic forward")
                                        : new PrintCommand("Running FL rotate quasistatic backward"),
                                rotateRoutine[0].quasistatic(direction))),
                        Map.entry(SysIdTest.FR_ROT, new ParallelCommandGroup(
                                direction == SysIdRoutine.Direction.kForward
                                        ? new PrintCommand("Running FR rotate quasistatic forward")
                                        : new PrintCommand("Running FR rotate quasistatic backward"),
                                rotateRoutine[1].quasistatic(direction))),
                        Map.entry(SysIdTest.BL_ROT, new ParallelCommandGroup(
                                direction == SysIdRoutine.Direction.kForward
                                        ? new PrintCommand("Running BL rotate quasistatic forward")
                                        : new PrintCommand("Running BL rotate quasistatic backward"),
                                rotateRoutine[2].quasistatic(direction))),
                        Map.entry(SysIdTest.BR_ROT, new ParallelCommandGroup(
                                direction == SysIdRoutine.Direction.kForward
                                        ? new PrintCommand("Running BR rotate quasistatic forward")
                                        : new PrintCommand("Running BR rotate quasistatic backward"),
                                rotateRoutine[3].quasistatic(direction)))

                // //TURN
                // Map.entry(SysIdTest.FLBR_TURN, new ParallelCommandGroup(
                // direction == SysIdRoutine.Direction.kForward ?
                // new PrintCommand("Running fL-bR turn quasistatic forward") :
                // new PrintCommand("Running fL-bR turn quasistatic backward"),
                // flbrTurn.quasistatic(direction)
                // )),
                // Map.entry(SysIdTest.FRBL_TURN, new ParallelCommandGroup(
                // direction == SysIdRoutine.Direction.kForward ?
                // new PrintCommand("Running fR-bL turn quasistatic forward") :
                // new PrintCommand("Running fR-bL turn quasistatic backward"),
                // frblTurn.quasistatic(direction)
                // )),
                // Map.entry(SysIdTest.ALL_TURN, new ParallelCommandGroup(
                // direction == SysIdRoutine.Direction.kForward ?
                // new PrintCommand("Running all turn quasistatic forward") :
                // new PrintCommand("Running all turn quasistatic backward"),
                // allWheelsTurn.quasistatic(direction)
                // ))
                ),
                this::selector);
    }

    // public Command sysIdDynamic(SysIdRoutine.Direction direction, int
    // frontorback) {
    // switch(frontorback) {
    // case 0:
    // return frontOnlyDrive.dynamic(direction);
    // case 1:
    // return backOnlyDrive.dynamic(direction);
    // case 2:
    // return allWheelsDrive.dynamic(direction);
    // }
    // return new PrintCommand("Invalid Command");
    // }
    private Command allTheSYSID(SysIdRoutine.Direction direction) {
        return new SequentialCommandGroup(
                frontOnlyDriveRoutine.dynamic(direction),
                backOnlyDriveRoutine.dynamic(direction),
                allWheelsDriveRoutine.dynamic(direction),
                rotateRoutine[0].dynamic(direction),
                rotateRoutine[1].dynamic(direction),
                rotateRoutine[2].dynamic(direction),
                rotateRoutine[3].dynamic(direction),

                frontOnlyDriveRoutine.quasistatic(direction),
                backOnlyDriveRoutine.quasistatic(direction),
                allWheelsDriveRoutine.quasistatic(direction),
                rotateRoutine[0].quasistatic(direction),
                rotateRoutine[1].quasistatic(direction),
                rotateRoutine[2].quasistatic(direction),
                rotateRoutine[3].quasistatic(direction));
    }

    /**
     * Makes sysId to run for both directions
     * @return Command to run sysId
     */
    public Command allTheSYSID() {
        return new SequentialCommandGroup(
                allTheSYSID(SysIdRoutine.Direction.kForward),
                allTheSYSID(SysIdRoutine.Direction.kReverse));
    }
    /**
     * Makes sysId to find feedforward and pid values for drivetrain
     * @param direction SysIdRoutine.Direction.kForward or kReverse
     * @return Command to run sysID
     */
    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return new SelectCommand<>(
    //             Map.ofEntries(
    //                     // DRIVE
    //                     Map.entry(SysIdTest.FRONT_DRIVE, new ParallelCommandGroup(
    //                             direction == SysIdRoutine.Direction.kForward
    //                                     ? new PrintCommand("Running front only dynamic forward")
    //                                     : new PrintCommand("Running front only dynamic backward"),
    //                             frontOnlyDriveRoutine.dynamic(direction))),
    //                     Map.entry(SysIdTest.BACK_DRIVE, new ParallelCommandGroup(
    //                             direction == SysIdRoutine.Direction.kForward
    //                                     ? new PrintCommand("Running back only dynamic forward")
    //                                     : new PrintCommand("Running back only dynamic backward"),
    //                             backOnlyDriveRoutine.dynamic(direction))),
    //                     Map.entry(SysIdTest.ALL_DRIVE, new ParallelCommandGroup(
    //                             direction == SysIdRoutine.Direction.kForward
    //                                     ? new PrintCommand("Running all wheels dynamic forward")
    //                                     : new PrintCommand("Running all wheels dynamic backward"),
    //                             allWheelsDriveRoutine.dynamic(direction))),
    //                     // ROTATE
    //                     Map.entry(SysIdTest.FL_ROT, new ParallelCommandGroup(
    //                             direction == SysIdRoutine.Direction.kForward
    //                                     ? new PrintCommand("Running FL rotate dynamic forward")
    //                                     : new PrintCommand("Running FL rotate dynamic backward"),
    //                             rotateRoutine[0].dynamic(direction))),
    //                     Map.entry(SysIdTest.FR_ROT, new ParallelCommandGroup(
    //                             direction == SysIdRoutine.Direction.kForward
    //                                     ? new PrintCommand("Running FR rotate dynamic forward")
    //                                     : new PrintCommand("Running FR rotate dynamic backward"),
    //                             rotateRoutine[1].dynamic(direction))),
    //                     Map.entry(SysIdTest.BL_ROT, new ParallelCommandGroup(
    //                             direction == SysIdRoutine.Direction.kForward
    //                                     ? new PrintCommand("Running BL rotate dynamic forward")
    //                                     : new PrintCommand("Running BL rotate dynamic backward"),
    //                             rotateRoutine[2].dynamic(direction))),
    //                     Map.entry(SysIdTest.BR_ROT, new ParallelCommandGroup(
    //                             direction == SysIdRoutine.Direction.kForward
    //                                     ? new PrintCommand("Running BR rotate dynamic forward")
    //                                     : new PrintCommand("Running BR rotate dynamic backward"),
    //                             rotateRoutine[3].dynamic(direction)))),
    //             this::selector);
    // }

    // #endregion
    
    /**
     * Sets all SwerveModules to point in a certain angle
     * @param angle in degrees
     */
    public void keepRotateMotorsAtDegrees(int angle) {
        for (SwerveModule module : modules) {
            module.turnPeriodic();
            module.move(0, angle);
        }
    }

    /**
     * Gets how fast the robot is spinning from gyro
     * @return degrees per second
     */
    public double getGyroRate() {
        return gyro.getRate();
    }
}
