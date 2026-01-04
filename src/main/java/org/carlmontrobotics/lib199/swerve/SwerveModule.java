package org.carlmontrobotics.lib199.swerve;


import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Pounds;

import java.util.function.Supplier;

import org.carlmontrobotics.lib199.MotorControllerType;
import org.mockito.internal.reporting.SmartPrinter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class that stores all the variables and methods applicaple to a single swerve module,
 * such as moving, getting encoder values, or configuring PID.
 */
public class SwerveModule implements Sendable {
    public enum ModuleType {FL, FR, BL, BR};

    //constants
    private final double neoStallTorqueNewtonMeters = 3.36;
    private final double neoFreeCurrentAmps = 1.3;
    private final double neoStallCurrentAmps = 166;
    private final double neoVortexStallTorqueNewtonMeters = 3.6;
    private final double neoVortexFreeCurrentAmps = 3.6;
    private final double neoVortexStallCurrentAmps = 211;
    private final double normalForceNewtons = 83.2 /* lbf */ * 4.4482 /* N/lbf */ / 4 /* numModules */;


    private SwerveConfig config;
    private SparkMaxConfig turnMaxConfig = new SparkMaxConfig();
    private SparkMaxConfig driveMaxConfig = new SparkMaxConfig();
    private SparkFlexConfig turnFlexConfig = new SparkFlexConfig();
    private SparkFlexConfig driveFlexConfig = new SparkFlexConfig();

    private ModuleType type;
    private SparkMax maxDrive, maxTurn;
    private SparkFlex flexDrive, flexTurn;
    private RelativeEncoder driveRelEnc, turnRelEnc;
    private boolean useFlexDrive, useFlexTurn; //false for max true for flex
    private CANcoder turnEncoder;
    private PIDController drivePIDController;
    private ProfiledPIDController turnPIDController;
    private TrapezoidProfile.Constraints turnConstraints;
    private double driveModifier, turnZeroDeg;
    private Supplier<Float> pitchDegSupplier, rollDegSupplier;
    private boolean reversed;
    private Timer timer;
    private SimpleMotorFeedforward forwardSimpleMotorFF, backwardSimpleMotorFF, turnSimpleMotorFeedforward;
    private double maxControllableAccerlationRps2;
    private double desiredSpeed, lastAngle, maxAchievableTurnVelocityRps, maxAchievableTurnAccelerationRps2, drivetoleranceMPerS, turnToleranceRot, angleDiffRot;

    private double turnSpeedCorrectionVolts, turnFFVolts, turnVolts;
    private double maxTurnVelocityWithoutTippingRps;

    MotorControllerType driveMotorType;
    MotorControllerType turnMotorType;
    
    public SwerveModule(SwerveConfig config, ModuleType type, SparkBase drive, SparkBase turn, CANcoder turnEncoder,
                        int arrIndex, Supplier<Float> pitchDegSupplier, Supplier<Float> rollDegSupplier) {
        String moduleString = type.toString();
        this.timer = new Timer();
        timer.start();
        this.config = config;
        this.type = type;
        this.maxDrive = drive;
        this.useFlexDrive = false;
        this.driveRelEnc = maxDrive.getEncoder();

        driveMaxConfig.inverted(config.driveInversion[arrIndex]);
        turnMaxConfig.inverted(config.turnInversion[arrIndex]);

        double drivePositionFactor = config.wheelDiameterMeters * Math.PI / config.driveGearing;
        final double driveVelocityFactor = drivePositionFactor / 60;
        driveMaxConfig.encoder
            .positionConversionFactor(drivePositionFactor)
            .velocityConversionFactor(driveVelocityFactor);

        maxControllableAccerlationRps2 = 0;
        double wheelTorqueLimitNewtonMeters = normalForceNewtons * config.mu * config.wheelDiameterMeters / 2;
        double motorTorqueLimitNewtonMeters = wheelTorqueLimitNewtonMeters / config.driveGearing;
        double currentLimitAmps = neoFreeCurrentAmps + 2*motorTorqueLimitNewtonMeters / neoStallTorqueNewtonMeters * (neoStallCurrentAmps-neoFreeCurrentAmps);
        // SmartDashboard.putNumber(type.toString() + " current limit (amps)", currentLimitAmps);
        driveMaxConfig.smartCurrentLimit(Math.min(50, (int)currentLimitAmps));
        
        this.forwardSimpleMotorFF = new SimpleMotorFeedforward(config.kForwardVolts[arrIndex],
                                                                config.kForwardVels[arrIndex],
                                                                config.kForwardAccels[arrIndex]);
        this.backwardSimpleMotorFF = new SimpleMotorFeedforward(config.kBackwardVolts[arrIndex],
                                                                config.kBackwardVels[arrIndex],
                                                                config.kBackwardAccels[arrIndex]);

        drivePIDController = new PIDController(config.drivekP[arrIndex],
                                               config.drivekI[arrIndex],
                                               config.drivekD[arrIndex]);
        
        /* offset for 1 relative encoder count */
        drivetoleranceMPerS = (1.0 
            / (double)(maxDrive.configAccessor.encoder.getCountsPerRevolution()) * drivePositionFactor) 
            / Units.millisecondsToSeconds(maxDrive.configAccessor.signals.getPrimaryEncoderPositionPeriodMs() * maxDrive.configAccessor.encoder.getQuadratureAverageDepth());
        drivePIDController.setTolerance(drivetoleranceMPerS);


        this.maxTurn = turn;
        this.useFlexTurn = false;
        this.turnRelEnc = maxTurn.getEncoder();

        this.turnSimpleMotorFeedforward = new SimpleMotorFeedforward(config.turnkS[arrIndex],
                                                                     config.turnkV[arrIndex],
                                                                     config.turnkA[arrIndex]);

        // Voltage = kS + kV * velocity + kA * acceleration
        // Assume cruising at maximum velocity --> 12 = kS + kV * max velocity + kA * 0 --> max velocity = (12 - kS) / kV
        // Limit the velocity by a factor of 0.5
        maxAchievableTurnVelocityRps = 0.5 * turnSimpleMotorFeedforward.maxAchievableVelocity(12.0, 0);
        maxTurnVelocityWithoutTippingRps = maxAchievableTurnVelocityRps;
        // Assume accelerating while at limited speed --> 12 = kS + kV * limited speed + kA * acceleration
        // acceleration = (12 - kS - kV * limiedSpeed) / kA
        turnToleranceRot = Units.degreesToRotations(3 * 360/4096.0); /* degree offset for 3 CANCoder counts */
        maxAchievableTurnAccelerationRps2 = 0.5 * turnSimpleMotorFeedforward.maxAchievableAcceleration(12.0, maxAchievableTurnVelocityRps);

        turnConstraints = new TrapezoidProfile.Constraints(maxAchievableTurnVelocityRps, maxAchievableTurnAccelerationRps2);
        lastAngle = 0.0;
        turnPIDController = new ProfiledPIDController(
            config.turnkP[arrIndex], 
            config.turnkI[arrIndex],
            config.turnkD[arrIndex],
            turnConstraints);
        turnPIDController.enableContinuousInput(-.5, .5);
        turnPIDController.setTolerance(turnToleranceRot);

        this.driveModifier = config.driveModifier;
        this.reversed = config.reversed[arrIndex];
        this.turnZeroDeg = config.turnZeroDeg[arrIndex];
        
        CANcoderConfiguration CANconfig = new CANcoderConfiguration();
        CANconfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = .5;
        this.turnEncoder = turnEncoder;
        // CANconfig.MagnetSensor.MagnetOffset=-turnZeroDeg; //done in getModuleAngle.
        this.turnEncoder.getConfigurator().apply(CANconfig);

        turnPIDController.reset(getModuleAngle());

        this.rollDegSupplier = rollDegSupplier;
        this.pitchDegSupplier = pitchDegSupplier;

        SendableRegistry.addLW(this, "SwerveModule", type.toString());

        //do stuff here
        maxDrive.configure(driveMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        maxTurn.configure(turnMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public SwerveModule(SwerveConfig config, ModuleType type, SparkMax drive, SparkFlex turn, CANcoder turnEncoder,
                        int arrIndex, Supplier<Float> pitchDegSupplier, Supplier<Float> rollDegSupplier) {
        String moduleString = type.toString();
        this.timer = new Timer();
        timer.start();
        this.config = config;
        this.type = type;
        this.maxDrive = drive;
        this.useFlexDrive = false;
        this.driveRelEnc = maxDrive.getEncoder();

        driveMaxConfig.inverted(config.driveInversion[arrIndex]);
        turnFlexConfig.inverted(config.turnInversion[arrIndex]);

        double drivePositionFactor = config.wheelDiameterMeters * Math.PI / config.driveGearing;
        final double driveVelocityFactor = drivePositionFactor / 60;
        driveMaxConfig.encoder
            .positionConversionFactor(drivePositionFactor)
            .velocityConversionFactor(driveVelocityFactor)
            .quadratureAverageDepth(2);

        maxControllableAccerlationRps2 = 0;
        double wheelTorqueLimitNewtonMeters = normalForceNewtons * config.mu * config.wheelDiameterMeters / 2;
        double motorTorqueLimitNewtonMeters = wheelTorqueLimitNewtonMeters / config.driveGearing;
        double currentLimitAmps = neoFreeCurrentAmps + 2*motorTorqueLimitNewtonMeters / neoStallTorqueNewtonMeters * (neoStallCurrentAmps-neoFreeCurrentAmps);
        // SmartDashboard.putNumber(type.toString() + " current limit (amps)", currentLimitAmps);
        driveMaxConfig.smartCurrentLimit(Math.min(50, (int)currentLimitAmps));
        
        this.forwardSimpleMotorFF = new SimpleMotorFeedforward(config.kForwardVolts[arrIndex],
                                                                config.kForwardVels[arrIndex],
                                                                config.kForwardAccels[arrIndex]);
        this.backwardSimpleMotorFF = new SimpleMotorFeedforward(config.kBackwardVolts[arrIndex],
                                                                config.kBackwardVels[arrIndex],
                                                                config.kBackwardAccels[arrIndex]);

        drivePIDController = new PIDController(config.drivekP[arrIndex],
                                               config.drivekI[arrIndex],
                                               config.drivekD[arrIndex]);
        
        /* offset for 1 relative encoder count */
        drivetoleranceMPerS = (1.0 
            / (double)(maxDrive.configAccessor.encoder.getCountsPerRevolution()) * drivePositionFactor) 
            / Units.millisecondsToSeconds(maxDrive.configAccessor.signals.getPrimaryEncoderPositionPeriodMs() * maxDrive.configAccessor.encoder.getQuadratureAverageDepth());
        drivePIDController.setTolerance(drivetoleranceMPerS);


        this.flexTurn = turn;
        this.useFlexTurn = true;
        this.turnRelEnc = flexTurn.getEncoder();

        this.turnSimpleMotorFeedforward = new SimpleMotorFeedforward(config.turnkS[arrIndex],
                                                                     config.turnkV[arrIndex],
                                                                     config.turnkA[arrIndex]);

        // Voltage = kS + kV * velocity + kA * acceleration
        // Assume cruising at maximum velocity --> 12 = kS + kV * max velocity + kA * 0 --> max velocity = (12 - kS) / kV
        // Limit the velocity by a factor of 0.5
        maxAchievableTurnVelocityRps = 0.5 * turnSimpleMotorFeedforward.maxAchievableVelocity(12.0, 0);
        maxTurnVelocityWithoutTippingRps = maxAchievableTurnVelocityRps;
        // Assume accelerating while at limited speed --> 12 = kS + kV * limited speed + kA * acceleration
        // acceleration = (12 - kS - kV * limiedSpeed) / kA
        turnToleranceRot = Units.degreesToRotations(3 * 360/4096.0); /* degree offset for 3 CANCoder counts */
        maxAchievableTurnAccelerationRps2 = 0.5 * turnSimpleMotorFeedforward.maxAchievableAcceleration(12.0, maxAchievableTurnVelocityRps);

        turnConstraints = new TrapezoidProfile.Constraints(maxAchievableTurnVelocityRps, maxAchievableTurnAccelerationRps2);
        lastAngle = 0.0;
        turnPIDController = new ProfiledPIDController(
            config.turnkP[arrIndex], 
            config.turnkI[arrIndex],
            config.turnkD[arrIndex],
            turnConstraints);
        turnPIDController.enableContinuousInput(-.5, .5);
        turnPIDController.setTolerance(turnToleranceRot);

        this.driveModifier = config.driveModifier;
        this.reversed = config.reversed[arrIndex];
        this.turnZeroDeg = config.turnZeroDeg[arrIndex];
        
        CANcoderConfiguration CANconfig = new CANcoderConfiguration();
        CANconfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = .5;
        this.turnEncoder = turnEncoder;
        // CANconfig.MagnetSensor.MagnetOffset=-turnZeroDeg; //done in getModuleAngle.
        this.turnEncoder.getConfigurator().apply(CANconfig);

        turnPIDController.reset(getModuleAngle());

        this.rollDegSupplier = rollDegSupplier;
        this.pitchDegSupplier = pitchDegSupplier;

        SendableRegistry.addLW(this, "SwerveModule", type.toString());

        //do stuff here
        maxDrive.configure(driveMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        flexTurn.configure(turnFlexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public SwerveModule(SwerveConfig config, ModuleType type, SparkFlex drive, SparkMax turn, CANcoder turnEncoder,
                        int arrIndex, Supplier<Float> pitchDegSupplier, Supplier<Float> rollDegSupplier) {
        String moduleString = type.toString();
        this.timer = new Timer();
        timer.start();
        this.config = config;
        this.type = type;
        this.flexDrive = drive;
        this.useFlexDrive = true;
        this.driveRelEnc = maxDrive.getEncoder();

        driveFlexConfig.inverted(config.driveInversion[arrIndex]);
        turnMaxConfig.inverted(config.turnInversion[arrIndex]);

        double drivePositionFactor = config.wheelDiameterMeters * Math.PI / config.driveGearing;
        final double driveVelocityFactor = drivePositionFactor / 60;
        driveFlexConfig.encoder
            .positionConversionFactor(drivePositionFactor)
            .velocityConversionFactor(driveVelocityFactor);

        maxControllableAccerlationRps2 = 0;
        double wheelTorqueLimitNewtonMeters = normalForceNewtons * config.mu * config.wheelDiameterMeters / 2;
        double motorTorqueLimitNewtonMeters = wheelTorqueLimitNewtonMeters / config.driveGearing;
        double currentLimitAmps = neoVortexFreeCurrentAmps + 2*motorTorqueLimitNewtonMeters / neoVortexStallCurrentAmps * (neoVortexStallCurrentAmps-neoVortexFreeCurrentAmps);
        // SmartDashboard.putNumber(type.toString() + " current limit (amps)", currentLimitAmps);
        driveFlexConfig.smartCurrentLimit(Math.min(50, (int)currentLimitAmps));
        
        this.forwardSimpleMotorFF = new SimpleMotorFeedforward(config.kForwardVolts[arrIndex],
                                                                config.kForwardVels[arrIndex],
                                                                config.kForwardAccels[arrIndex]);
        this.backwardSimpleMotorFF = new SimpleMotorFeedforward(config.kBackwardVolts[arrIndex],
                                                                config.kBackwardVels[arrIndex],
                                                                config.kBackwardAccels[arrIndex]);

        drivePIDController = new PIDController(config.drivekP[arrIndex],
                                               config.drivekI[arrIndex],
                                               config.drivekD[arrIndex]);
        
        /* offset for 1 relative encoder count */
        drivetoleranceMPerS = (1.0 
            / (double)(flexDrive.configAccessor.encoder.getCountsPerRevolution()) * drivePositionFactor) 
            / Units.millisecondsToSeconds(flexDrive.configAccessor.signals.getPrimaryEncoderPositionPeriodMs() * flexDrive.configAccessor.encoder.getQuadratureAverageDepth());
        drivePIDController.setTolerance(drivetoleranceMPerS);


        this.maxTurn = turn;
        this.useFlexTurn = false;
        this.turnRelEnc = maxTurn.getEncoder();

        this.turnSimpleMotorFeedforward = new SimpleMotorFeedforward(config.turnkS[arrIndex],
                                                                     config.turnkV[arrIndex],
                                                                     config.turnkA[arrIndex]);

        // Voltage = kS + kV * velocity + kA * acceleration
        // Assume cruising at maximum velocity --> 12 = kS + kV * max velocity + kA * 0 --> max velocity = (12 - kS) / kV
        // Limit the velocity by a factor of 0.5
        maxAchievableTurnVelocityRps = 0.5 * turnSimpleMotorFeedforward.maxAchievableVelocity(12.0, 0);
        maxTurnVelocityWithoutTippingRps = maxAchievableTurnVelocityRps;
        // Assume accelerating while at limited speed --> 12 = kS + kV * limited speed + kA * acceleration
        // acceleration = (12 - kS - kV * limiedSpeed) / kA
        turnToleranceRot = Units.degreesToRotations(3 * 360/4096.0); /* degree offset for 3 CANCoder counts */
        maxAchievableTurnAccelerationRps2 = 0.5 * turnSimpleMotorFeedforward.maxAchievableAcceleration(12.0, maxAchievableTurnVelocityRps);

        turnConstraints = new TrapezoidProfile.Constraints(maxAchievableTurnVelocityRps, maxAchievableTurnAccelerationRps2);
        lastAngle = 0.0;
        turnPIDController = new ProfiledPIDController(
            config.turnkP[arrIndex], 
            config.turnkI[arrIndex],
            config.turnkD[arrIndex],
            turnConstraints);
        turnPIDController.enableContinuousInput(-.5, .5);
        turnPIDController.setTolerance(turnToleranceRot);

        this.driveModifier = config.driveModifier;
        this.reversed = config.reversed[arrIndex];
        this.turnZeroDeg = config.turnZeroDeg[arrIndex];
        
        CANcoderConfiguration CANconfig = new CANcoderConfiguration();
        CANconfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = .5;
        this.turnEncoder = turnEncoder;
        // CANconfig.MagnetSensor.MagnetOffset=-turnZeroDeg; //done in getModuleAngle.
        this.turnEncoder.getConfigurator().apply(CANconfig);

        turnPIDController.reset(getModuleAngle());

        this.rollDegSupplier = rollDegSupplier;
        this.pitchDegSupplier = pitchDegSupplier;

        SendableRegistry.addLW(this, "SwerveModule", type.toString());

        //do stuff here
        flexDrive.configure(driveFlexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        maxTurn.configure(turnMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public SwerveModule(SwerveConfig config, ModuleType type, SparkFlex drive, SparkFlex turn, CANcoder turnEncoder,
                        int arrIndex, Supplier<Float> pitchDegSupplier, Supplier<Float> rollDegSupplier) {
        String moduleString = type.toString();
        this.timer = new Timer();
        timer.start();
        this.config = config;
        this.type = type;
        this.flexDrive = drive;
        this.useFlexDrive = true;
        this.driveRelEnc = maxDrive.getEncoder();

        driveFlexConfig.inverted(config.driveInversion[arrIndex]);
        turnFlexConfig.inverted(config.turnInversion[arrIndex]);

        double drivePositionFactor = config.wheelDiameterMeters * Math.PI / config.driveGearing;
        final double driveVelocityFactor = drivePositionFactor / 60;
        driveFlexConfig.encoder
            .positionConversionFactor(drivePositionFactor)
            .velocityConversionFactor(driveVelocityFactor);

        maxControllableAccerlationRps2 = 0;
        double wheelTorqueLimitNewtonMeters = normalForceNewtons * config.mu * config.wheelDiameterMeters / 2;
        double motorTorqueLimitNewtonMeters = wheelTorqueLimitNewtonMeters / config.driveGearing;
        double currentLimitAmps = neoVortexFreeCurrentAmps + 2*motorTorqueLimitNewtonMeters / neoVortexStallCurrentAmps * (neoVortexStallCurrentAmps-neoVortexFreeCurrentAmps);
        // SmartDashboard.putNumber(type.toString() + " current limit (amps)", currentLimitAmps);
        driveFlexConfig.smartCurrentLimit(Math.min(50, (int)currentLimitAmps));
        
        this.forwardSimpleMotorFF = new SimpleMotorFeedforward(config.kForwardVolts[arrIndex],
                                                                config.kForwardVels[arrIndex],
                                                                config.kForwardAccels[arrIndex]);
        this.backwardSimpleMotorFF = new SimpleMotorFeedforward(config.kBackwardVolts[arrIndex],
                                                                config.kBackwardVels[arrIndex],
                                                                config.kBackwardAccels[arrIndex]);

        drivePIDController = new PIDController(config.drivekP[arrIndex],
                                               config.drivekI[arrIndex],
                                               config.drivekD[arrIndex]);
        
        /* offset for 1 relative encoder count */
        drivetoleranceMPerS = (1.0 
            / (double)(flexDrive.configAccessor.encoder.getCountsPerRevolution()) * drivePositionFactor) 
            / Units.millisecondsToSeconds(flexDrive.configAccessor.signals.getPrimaryEncoderPositionPeriodMs() * flexDrive.configAccessor.encoder.getQuadratureAverageDepth());
        drivePIDController.setTolerance(drivetoleranceMPerS);


        this.flexTurn = turn;
        this.useFlexTurn = true;
        this.turnRelEnc = flexTurn.getEncoder();

        this.turnSimpleMotorFeedforward = new SimpleMotorFeedforward(config.turnkS[arrIndex],
                                                                     config.turnkV[arrIndex],
                                                                     config.turnkA[arrIndex]);

        // Voltage = kS + kV * velocity + kA * acceleration
        // Assume cruising at maximum velocity --> 12 = kS + kV * max velocity + kA * 0 --> max velocity = (12 - kS) / kV
        // Limit the velocity by a factor of 0.5
        maxAchievableTurnVelocityRps = 0.5 * turnSimpleMotorFeedforward.maxAchievableVelocity(12.0, 0);
        maxTurnVelocityWithoutTippingRps = maxAchievableTurnVelocityRps;
        // Assume accelerating while at limited speed --> 12 = kS + kV * limited speed + kA * acceleration
        // acceleration = (12 - kS - kV * limiedSpeed) / kA
        turnToleranceRot = Units.degreesToRotations(3 * 360/4096.0); /* degree offset for 3 CANCoder counts */
        maxAchievableTurnAccelerationRps2 = 0.5 * turnSimpleMotorFeedforward.maxAchievableAcceleration(12.0, maxAchievableTurnVelocityRps);

        turnConstraints = new TrapezoidProfile.Constraints(maxAchievableTurnVelocityRps, maxAchievableTurnAccelerationRps2);
        lastAngle = 0.0;
        turnPIDController = new ProfiledPIDController(
            config.turnkP[arrIndex], 
            config.turnkI[arrIndex],
            config.turnkD[arrIndex],
            turnConstraints);
        turnPIDController.enableContinuousInput(-.5, .5);
        turnPIDController.setTolerance(turnToleranceRot);

        this.driveModifier = config.driveModifier;
        this.reversed = config.reversed[arrIndex];
        this.turnZeroDeg = config.turnZeroDeg[arrIndex];
        
        CANcoderConfiguration CANconfig = new CANcoderConfiguration();
        CANconfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = .5;
        this.turnEncoder = turnEncoder;
        // CANconfig.MagnetSensor.MagnetOffset=-turnZeroDeg; //done in getModuleAngle.
        this.turnEncoder.getConfigurator().apply(CANconfig);

        turnPIDController.reset(getModuleAngle());

        this.rollDegSupplier = rollDegSupplier;
        this.pitchDegSupplier = pitchDegSupplier;

        SendableRegistry.addLW(this, "SwerveModule", type.toString());

        //do stuff here
        flexDrive.configure(driveFlexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        flexTurn.configure(turnFlexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public ModuleType getType() {
        return type;
    }

    private double prevTurnVelocity = 0;
    
    public void periodic() {
        drivePeriodic();
        // updateSmartDashboard();
        turnPeriodic();
    }

    public void drivePeriodic() {
        String moduleString = type.toString();
        double actualSpeed = getCurrentSpeed();
        double extraAccel = calculateAntiGravitationalA(pitchDegSupplier.get(), rollDegSupplier.get());
        double targetVoltage = (actualSpeed >= 0 ? forwardSimpleMotorFF : backwardSimpleMotorFF)
            .calculateWithVelocities(
                actualSpeed, 
                desiredSpeed + extraAccel * TimedRobot.kDefaultPeriod  //m/s + ( m/s^2 * s )
            );
        // Use robot characterization as a simple physical model to account for internal resistance, frcition, etc.
        // Add a PID adjustment for error correction (also "drives" the actual speed to the desired speed)
        double pidVolts = drivePIDController.calculate(actualSpeed, desiredSpeed);
        // SmartDashboard.putNumber(moduleString + "Actual Speed", actualSpeed);
        // SmartDashboard.putNumber(moduleString + "Desired Speed", desiredSpeed);
        if (drivePIDController.atSetpoint()) {
            pidVolts = 0;
        }
        targetVoltage += pidVolts;
        // SmartDashboard.putBoolean(moduleString + " is within drive tolerance", drivePIDController.atSetpoint());
        // SmartDashboard.putNumber(moduleString + " pidVolts", pidVolts);
        double appliedVoltage = MathUtil.clamp(targetVoltage, -12, 12);
        // SmartDashboard.putNumber(moduleString + " appliedVoltage", appliedVoltage);
        if (useFlexDrive) {
            flexDrive.setVoltage(appliedVoltage);
        }
        else {
            maxDrive.setVoltage(appliedVoltage);
        }
    }

    public void turnPeriodic() {
        String moduleString = type.toString();
        // Turn Control
        {
            double measuredAngleRots = Units.degreesToRotations(getModuleAngle());
            TrapezoidProfile.State goal = turnPIDController.getGoal();
            // SmartDashboard.putNumber(moduleString + " goaltrap.position", goal.position);
            // SmartDashboard.putNumber(moduleString + " goaltrap.velocity", goal.velocity);
            goal = new TrapezoidProfile.State(goal.position, goal.velocity);

            double period = turnPIDController.getPeriod();
            double optimalTurnVelocityRps = Math.abs(MathUtil.inputModulus(goal.position-measuredAngleRots, -.5, .5))/period;
            
            double maxOverShootDegree = 1;
            // maxOverShootDegree = SmartDashboard.getNumber("maxOverShootDegree",maxOverShootDegree);
            int numPeriods = 1;
            // numPeriods =(int) SmartDashboard.getNumber("num periods",numPeriods);
            maxControllableAccerlationRps2 = (2*(maxOverShootDegree/360))/Math.pow(period*numPeriods,2);
            setMaxTurnVelocity(Math.min(Math.min(maxAchievableTurnVelocityRps, optimalTurnVelocityRps), maxTurnVelocityWithoutTippingRps));
            // SmartDashboard.putNumber("maxTurnVelocityWithoutTippingRps", maxTurnVelocityWithoutTippingRps);
            // SmartDashboard.putNumber("maxAcheivableTurnVelcoityRPS", maxAchievableTurnVelocityRps);
            // SmartDashboard.putNumber("optimalTurnVelocityRPS", optimalTurnVelocityRps);
            
            turnSpeedCorrectionVolts = turnPIDController.calculate(measuredAngleRots);
            TrapezoidProfile.State state = turnPIDController.getSetpoint();

            // SmartDashboard.putNumber("previous turn Velocity", prevTurnVelocity);
            // SmartDashboard.putNumber("state velocity",state.velocity);
            turnFFVolts = turnSimpleMotorFeedforward.calculate(state.velocity);//(state.velocity-prevTurnVelocity) / period);
            turnVolts = turnFFVolts + turnSpeedCorrectionVolts;
            if (!turnPIDController.atGoal()) {
                if (useFlexTurn) {
                    flexTurn.setVoltage(MathUtil.clamp(turnVolts, -12.0, 12.0));
                }
                else {
                    maxTurn.setVoltage(MathUtil.clamp(turnVolts, -12.0, 12.0));
                }
            } else {
                if (useFlexTurn) {
                    flexTurn.setVoltage(turnSimpleMotorFeedforward.calculate(goal.velocity));
                }
                else {
                    maxTurn.setVoltage(turnSimpleMotorFeedforward.calculate(goal.velocity));
                }
            }
            prevTurnVelocity = state.velocity;
        }   
    }

    /**
     * Move the module to a specified ang
     * le and drive at a specified speed.
     * @param speedMps   The desired speed in m/s.
     * @param angle   The desired angle in degrees.
     */
    public void move(double speedMps, double angle) {
        setSpeed(speedMps);
        if(speedMps != 0.0) {
            //SmartDashboard.putNumber(moduleString + " Target Angle (deg)", angle);
            angle = MathUtil.inputModulus(Units.degreesToRotations(angle), -0.5, 0.5);
            setAngle(angle);
        }
    }

    /**
     * Calculates the amount of additional forward accelration needed to combat gravity
     * @param gyroPitchDeg Pitch of gyro in degrees
     * @param gyroRollDeg  Roll of gyro in degrees
     * @return the amount of additional forward accelration needed to combat gravity in m/s^2
     */
    private double calculateAntiGravitationalA(Float gyroPitchDeg, Float gyroRollDeg)
    {
        double moduleAngle = getModuleAngle() * Math.PI / 180; // In Radians
        double moduleRollComponent = Math.sin(moduleAngle);
        double modulePitchComponent = Math.cos(moduleAngle);
        double g = 9.81; //meters per second squared
        // gravitationalA is estimated to work for small angles, not 100% accurate at large angles
        double antiGravitationalA = g * (modulePitchComponent * Math.sin(Math.PI * gyroPitchDeg / 180) - moduleRollComponent * Math.sin(Math.PI * gyroRollDeg / 180));
        // SmartDashboard.putNumber("AntiGravitational acceleration", antiGravitationalA);
        return antiGravitationalA;
    }

    /**
     * Sets the speed for the drive motor controller.
     * @param speedMps     The desired speed in meters per second.
     */
    private void setSpeed(double speedMps) {
        desiredSpeed = speedMps * driveModifier;
    }

    /**
     * Sets the angle for the turn motor controller.
     * @param angle     The desired angle, between -1/2 rotations clockwise and 1/2 rotations counterclockwise.
     */
    private void setAngle(double angle) {
        double deltaTime = timer.get();
        // SmartDashboard.putNumber("config.autoCentripetalAccel", config.autoCentripetalAccel);
        timer.reset();
        timer.start();
        double maxDeltaTheta = Math.atan2(deltaTime*config.autoCentripetalAccel,(Math.abs(getCurrentSpeed())));
        maxTurnVelocityWithoutTippingRps = Units.degreesToRotations(maxDeltaTheta*180/Math.PI)/deltaTime;
        //SmartDashboard.putNumber(moduleString + "Target Angle:", 360 * angle * (reversed ? -1 : 1));

        // Find the minimum distance to travel from lastAngle to angle and determine the
        // correct direction to trvel the minimum distance. This is used in order to accurately
        // calculate the goal velocity.
        angleDiffRot = MathUtil.inputModulus(angle - lastAngle, -1/2, 1/2);
        // SmartDashboard.putNumber(moduleString + " angleDiff (deg)", angleDiff);

        turnPIDController.setGoal(new TrapezoidProfile.State(angle * (reversed ? -1 : 1), 0));
        lastAngle = angle;
    }

    /**
     * Gets the current angle of the module
     * @return module angle in degrees
     */
    public double getModuleAngle() {
        return MathUtil.inputModulus(Units.rotationsToDegrees(turnEncoder.getAbsolutePosition().getValueAsDouble()) - turnZeroDeg, -180, 180);
    }

    /**
     * Gets the current state (speed and angle) of this module.
     * @return A SwerveModuleState object representing the speed and angle of the module.
     */
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getCurrentSpeed(), Rotation2d.fromDegrees(getModuleAngle()));
    }

    /**
     * Gets the current position (distance and angle) of this module.
     * @return A SwerveModulePosition object representing the distance and angle of the module.
     */
    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(getCurrentDistance(), Rotation2d.fromDegrees(getModuleAngle()));
    }

    public double getCurrentDistance() {
        return driveRelEnc.getPosition();
    }

    public double getCurrentSpeed() {
        return driveRelEnc.getVelocity();
    }

    /**
     * Updates SmartDashboard with information about this module.
     *
     * @deprecated Put this Sendable to SmartDashboard instead
     */
    @Deprecated
    public void updateSmartDashboard() {
        String moduleString = type.toString();
        // Display the position of the quadrature encoder.
        SmartDashboard.putNumber(moduleString + " Incremental Position", turnEncoder.getPosition().getValueAsDouble());
        // Display the position of the analog encoder.
        SmartDashboard.putNumber(moduleString + " Absolute Angle (deg)", Units.rotationsToDegrees(turnEncoder.getAbsolutePosition().getValueAsDouble()));
        // Display the module angle as calculated using the absolute encoder.
        SmartDashboard.putNumber(moduleString + " Turn Measured Pos (deg)", getModuleAngle());
        SmartDashboard.putNumber(moduleString + " Encoder Position", driveRelEnc.getPosition());
        // Display the speed that the robot thinks it is travelling at.
        SmartDashboard.putNumber(moduleString + " Current Speed", getCurrentSpeed());
        SmartDashboard.putBoolean(moduleString + " Drive is at Goal", drivePIDController.atSetpoint());
        SmartDashboard.putNumber(moduleString + " Turn Setpoint Pos (deg)", turnPIDController.getSetpoint().position);
        SmartDashboard.putNumber(moduleString + " Turn Setpoint Vel (dps)", turnPIDController.getSetpoint().velocity);
        SmartDashboard.putNumber(moduleString + " Turn Goal Pos (deg)", turnPIDController.getGoal().position);
        SmartDashboard.putNumber(moduleString + " Turn Goal Vel (dps)", turnPIDController.getGoal().velocity);
        //SmartDashboard.putNumber("Gyro Pitch", pitchDegSupplier.get());
        //SmartDashboard.putNumber("Gyro Roll", rollDegSupplier.get());
        SmartDashboard.putNumber(moduleString + "Antigravitational Acceleration", calculateAntiGravitationalA(pitchDegSupplier.get(), rollDegSupplier.get()));
        SmartDashboard.putBoolean(moduleString + " Turn is at Goal", turnPIDController.atGoal());
        
        SmartDashboard.putNumber(moduleString + "Turn PID Output (Volts)", turnSpeedCorrectionVolts);
        SmartDashboard.putNumber(moduleString + "Turn FF Output (Volts)", turnFFVolts);
        SmartDashboard.putNumber(moduleString + "Turn Total Output (Volts)", turnVolts);

        // tuning PID + FF values
        double drivekP = SmartDashboard.getNumber(moduleString + " Drive kP", drivePIDController.getP());
        if (drivePIDController.getP() != drivekP) {
            drivePIDController.setP(drivekP);
        }
        double drivekD = SmartDashboard.getNumber(moduleString + " Drive kD", drivePIDController.getD());
        if (drivePIDController.getD() != drivekD) {
            drivePIDController.setD(drivekD);
        }
        double driveTolerance = SmartDashboard.getNumber(moduleString + " Drive Tolerance", drivePIDController.getErrorTolerance());
        if (drivePIDController.getErrorTolerance() != driveTolerance) {
            drivePIDController.setTolerance(driveTolerance);
        }
        double drivekS = SmartDashboard.getNumber(moduleString + " Drive kS", forwardSimpleMotorFF.getKs());
        double drivekV = SmartDashboard.getNumber(moduleString + " Drive kV", forwardSimpleMotorFF.getKv());
        double drivekA = SmartDashboard.getNumber(moduleString + " Drive kA", forwardSimpleMotorFF.getKa());
        if (forwardSimpleMotorFF.getKs() != drivekS || forwardSimpleMotorFF.getKv() != drivekV || forwardSimpleMotorFF.getKa() != drivekA) {
            forwardSimpleMotorFF = new SimpleMotorFeedforward(drivekS, drivekV, drivekA);
            backwardSimpleMotorFF = new SimpleMotorFeedforward(drivekS, drivekV, drivekA);
        }
        double kP = SmartDashboard.getNumber(moduleString + " Swerve kP", turnPIDController.getP());
        if (turnPIDController.getP() != kP) {
            turnPIDController.setP(kP);
        }
        double kD = SmartDashboard.getNumber(moduleString + " Swerve kD", turnPIDController.getD());
        if (turnPIDController.getD() != kD) {
            turnPIDController.setD(kD);
        }
        double turnTolerance = SmartDashboard.getNumber(moduleString + " Swerve Turn Tolerance", turnToleranceRot);
        if (turnPIDController.getPositionTolerance() != turnTolerance) {
            turnPIDController.setTolerance(turnTolerance);
        }
        double kS = SmartDashboard.getNumber(moduleString + " Swerve kS", turnSimpleMotorFeedforward.getKs());
        double kV = SmartDashboard.getNumber(moduleString + " Swerve kV", turnSimpleMotorFeedforward.getKv());
        double kA = SmartDashboard.getNumber(moduleString + " Swerve kA", turnSimpleMotorFeedforward.getKa());
        if (turnSimpleMotorFeedforward.getKs() != kS || turnSimpleMotorFeedforward.getKv() != kV || turnSimpleMotorFeedforward.getKa() != kA) {
            turnSimpleMotorFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
            maxAchievableTurnVelocityRps = 0.5 * turnSimpleMotorFeedforward.maxAchievableVelocity(12.0, 0);
            maxAchievableTurnAccelerationRps2 = 0.5 * turnSimpleMotorFeedforward.maxAchievableAcceleration(12.0, maxAchievableTurnVelocityRps);
        }
    }

    public void toggleMode() {
        IdleMode driveIdleMode;
        IdleMode turnIdleMode;
        if (useFlexDrive) {
            driveIdleMode = flexDrive.configAccessor.getIdleMode();
        }
        else {
            driveIdleMode = maxDrive.configAccessor.getIdleMode();
        }
        if (useFlexTurn) {
            turnIdleMode = flexTurn.configAccessor.getIdleMode();
        }
        else {
            turnIdleMode = maxTurn.configAccessor.getIdleMode();
        }
        if (driveIdleMode == IdleMode.kBrake && turnIdleMode == IdleMode.kCoast) coast();
        else brake();
    }

    public void brake() {
        SparkBaseConfig maxConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake);
        SparkBaseConfig flexConfig = new SparkFlexConfig().idleMode(IdleMode.kBrake);
        if (useFlexDrive) {flexDrive.configure(flexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);}
        else {maxDrive.configure(maxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);}
        if (useFlexTurn) {flexTurn.configure(flexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);}
        else {maxTurn.configure(maxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);}
    }

    public void coast() {
        SparkBaseConfig maxConfig = new SparkMaxConfig().idleMode(IdleMode.kCoast);
        SparkBaseConfig flexConfig = new SparkFlexConfig().idleMode(IdleMode.kCoast);
        if (useFlexDrive) {flexDrive.configure(flexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);}
        else {maxDrive.configure(maxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);}
        if (useFlexTurn) {flexTurn.configure(flexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);}
        else {maxTurn.configure(maxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);}
    }

    /**
     * 
     * @param maxVel velocity in rot/s
     */
    public void setMaxTurnVelocity(double maxVel) {
        turnConstraints = new TrapezoidProfile.Constraints(maxVel, Math.min(maxControllableAccerlationRps2,maxAchievableTurnAccelerationRps2));
        SmartDashboard.putNumber("trap.maxvel", maxVel);
        SmartDashboard.putNumber("trap.maxAccel", turnConstraints.maxAcceleration);
        turnPIDController.setConstraints(turnConstraints);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setActuator(true);
        builder.setSafeState(() -> setSpeed(0));
        builder.setSmartDashboardType("SwerveModule");
        builder.addDoubleProperty("Incremental Position", () -> turnEncoder.getPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("Absolute Angle (deg)", () -> Units.rotationsToDegrees(turnEncoder.getAbsolutePosition().getValueAsDouble()), null);
        builder.addDoubleProperty("Turn Measured Pos (deg)", this::getModuleAngle, null);
        builder.addDoubleProperty("Encoder Position", driveRelEnc::getPosition, null);
        // Display the speed that the robot thinks it is travelling at.
        builder.addDoubleProperty("Current Speed", this::getCurrentSpeed, null);
        builder.addDoubleProperty("Turn Setpoint Pos (deg)", () -> turnPIDController.getSetpoint().position, null);
        builder.addDoubleProperty("Turn Setpoint Vel (dps)", () -> turnPIDController.getSetpoint().velocity, null);
        builder.addDoubleProperty("Turn Goal Pos (deg)", () -> turnPIDController.getGoal().position, null);
        builder.addDoubleProperty("Turn Goal Vel (dps)", () -> turnPIDController.getGoal().velocity, null);
        builder.addDoubleProperty("Antigravitational Acceleration", () -> calculateAntiGravitationalA(pitchDegSupplier.get(), rollDegSupplier.get()), null);
        builder.addBooleanProperty("Turn is at Goal", turnPIDController::atGoal, null);
        builder.addDoubleProperty("Error (deg)", turnPIDController::getPositionError, null);
        builder.addDoubleProperty("Desired Speed (mps)", drivePIDController::getSetpoint, null);
        builder.addDoubleProperty("Angle Diff (deg)", () -> angleDiffRot, null);

        builder.addDoubleProperty("Turn PID Output", () -> turnSpeedCorrectionVolts, null);
        builder.addDoubleProperty("Turn FF Output", () -> turnFFVolts, null);
        builder.addDoubleProperty("Turn Total Output", () -> turnVolts, null);
    }

    /**
     * Create and return a SwerveModuleSim that simulates the physics of this swerve module.
     * 
     * @param massOnWheel the mass on the wheel of this module (typically the mass of the robot divided by the number of modules)
     * @param turnGearing the gearing reduction between the turn motor and the module
     * @param turnMoiKgM2 the moment of inertia of the part of the module turned by the turn motor (in kg m^2)
     * @return a SwerveModuleSim that simulates the physics of this swerve module.
     */
    public SwerveModuleSim createSim(Mass massOnWheel, double turnGearing, double turnMoiKgM2) {
        double driveMoiKgM2 =  massOnWheel.in(Kilogram) * Math.pow(config.wheelDiameterMeters/2, 2);
        int driveId = useFlexDrive ? flexDrive.getDeviceId() : maxDrive.getDeviceId();
        int turnId = useFlexTurn ? flexTurn.getDeviceId() : maxTurn.getDeviceId();
        return new SwerveModuleSim(driveId, config.driveGearing, driveMoiKgM2, 
            turnId, turnEncoder.getDeviceID(), turnGearing, turnMoiKgM2);
    }

    /**
     * 
     * @return a SwerveModuleSim that simulates this swerve module assuming it is one of 4 MK4i modules on our 114 lb 2024 robot.
     */
    public SwerveModuleSim createSim() {
        return createSim(Pounds.of(114.0/4), 150.0/7, 0.0313);
    }
}
