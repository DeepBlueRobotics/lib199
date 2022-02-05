package frc.robot.lib;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class that stores all the variables and methods applicaple to a single swerve module,
 * such as moving, getting encoder values, or configuring PID.
 */
public class SwerveModule {
    public enum ModuleType {FL, FR, BL, BR};

    private SwerveConfig config;
    private ModuleType type;
    private String moduleString;
    private CANSparkMax drive, turn;
    private CANCoder turnEncoder;
    private PIDController drivePIDController;
    private ProfiledPIDController turnPIDController;
    private TrapezoidProfile.Constraints turnConstraints;
    private double driveModifier, maxSpeed, turnZero;
    private Supplier<Float> pitchDegSupplier, rollDegSupplier;
    private boolean reversed;
    private Timer timer;
    private SimpleMotorFeedforward forwardSimpleMotorFF, backwardSimpleMotorFF, turnSimpleMotorFeedforward;
    private double lastAngle, maxAchievableTurnVelocityDps, maxAchievableTurnAccelerationMps2, turnToleranceDeg;

    public SwerveModule(SwerveConfig config, ModuleType type, CANSparkMax drive, CANSparkMax turn, CANCoder turnEncoder, double driveModifier,
                        double maxSpeed, int arrIndex, Supplier<Float> pitchDegSupplier, Supplier<Float> rollDegSupplier) {
        //SmartDashboard.putNumber("Target Angle (deg)", 0.0);
        this.timer = new Timer();
        timer.start();

        this.config = config;
        this.type = type;
        this.drive = drive;

        switch (type) {
            case FL:
                moduleString = "FL";
                break;
            case FR:
                moduleString = "FR";
                break;
            case BL:
                moduleString = "BL";
                break;
            case BR:
                moduleString = "BR";
                break;
        }

        double positionConstant = config.wheelDiameterMeters * Math.PI / config.driveGearing;
        drive.setInverted(config.driveInversion[arrIndex]);
        drive.getEncoder().setPositionConversionFactor(positionConstant);
        drive.getEncoder().setVelocityConversionFactor(positionConstant / 60);

        final double normalForceNewtons = 83.2 /* lbf */ * 4.4482 /* N/lbf */ / 4 /* numModules */;
        double wheelTorqueLimitNewtonMeters = normalForceNewtons * config.mu * config.wheelDiameterMeters / 2;
        double motorTorqueLimitNewtonMeters = wheelTorqueLimitNewtonMeters / config.driveGearing;
        final double neoStallTorqueNewtonMeters = 3.36;
        final double neoFreeCurrentAmps = 1.3;
        final double neoStallCurrentAmps = 166;
        double currentLimitAmps = neoFreeCurrentAmps + 2*motorTorqueLimitNewtonMeters / neoStallTorqueNewtonMeters * (neoStallCurrentAmps-neoFreeCurrentAmps);
        SmartDashboard.putNumber(moduleString + " current limit (amps)", currentLimitAmps);
        drive.setSmartCurrentLimit((int)Math.min(50, currentLimitAmps));

        this.forwardSimpleMotorFF = new SimpleMotorFeedforward(config.kForwardVolts[arrIndex],
                                                                config.kForwardVels[arrIndex],
                                                                config.kForwardAccels[arrIndex]);
        this.backwardSimpleMotorFF = new SimpleMotorFeedforward(config.kBackwardVolts[arrIndex],
                                                                config.kBackwardVels[arrIndex],
                                                                config.kBackwardAccels[arrIndex]);

        drivePIDController = new PIDController(2 * config.drivekP[arrIndex],
                                               config.drivekI[arrIndex],
                                               config.drivekD[arrIndex]);
    

        //System.out.println("Velocity Constant: " + (positionConstant / 60));

        this.turn = turn;

        this.turnSimpleMotorFeedforward = new SimpleMotorFeedforward(config.turnkS[arrIndex],
                                                                     config.turnkV[arrIndex],
                                                                     config.turnkA[arrIndex]);

        // Voltage = kS + kV * velocity + kA * acceleration
        // Assume cruising at maximum velocity --> 12 = kS + kV * max velocity + kA * 0 --> max velocity = (12 - kS) / kV
        // Limit the velocity by a factor of 0.5
        maxAchievableTurnVelocityDps = 0.5 * turnSimpleMotorFeedforward.maxAchievableVelocity(12.0, 0);
        // Assume accelerating while at limited speed --> 12 = kS + kV * limited speed + kA * acceleration
        // acceleration = (12 - kS - kV * limiedSpeed) / kA
        turnToleranceDeg = 3 * 360/4096.0; /* degree offset for 3 CANCoder counts */
        maxAchievableTurnAccelerationMps2 = 0.5 * turnSimpleMotorFeedforward.maxAchievableAcceleration(12.0, maxAchievableTurnVelocityDps);
        turnConstraints = new TrapezoidProfile.Constraints(maxAchievableTurnVelocityDps, maxAchievableTurnAccelerationMps2);
        lastAngle = 0.0;
        turnPIDController = new ProfiledPIDController(config.turnkP[arrIndex], 
                                              config.turnkI[arrIndex],
                                              config.turnkD[arrIndex],
                                              turnConstraints);
        turnPIDController.enableContinuousInput(-180.0, 180.0);
        turnPIDController.setTolerance(turnToleranceDeg);

        this.turnEncoder = turnEncoder;
        this.turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        this.driveModifier = driveModifier;
        this.maxSpeed = maxSpeed;
        this.reversed = config.reversed[arrIndex];
        this.turnZero = config.turnZero[arrIndex];

        turnPIDController.reset(getModuleAngle());

        this.rollDegSupplier = rollDegSupplier;
        this.pitchDegSupplier = pitchDegSupplier;
    }

    private double prevTurnVelocity = 0;
    public void periodic() {
        double measuredAngleDegs = getModuleAngle();
        TrapezoidProfile.State goal = turnPIDController.getGoal();
        goal = new TrapezoidProfile.State(goal.position, goal.velocity);
        
        double period = turnPIDController.getPeriod();
        double optimalTurnVelocityDps = Math.abs(MathUtil.inputModulus(goal.position-measuredAngleDegs, -180, 180))/period;
        setMaxTurnVelocity(Math.min(maxAchievableTurnVelocityDps, optimalTurnVelocityDps));

        double turnSpeedCorrectionDps = turnPIDController.calculate(measuredAngleDegs) * turnSimpleMotorFeedforward.maxAchievableVelocity(12,0);
        TrapezoidProfile.State state = turnPIDController.getSetpoint();
        double turnVolts = turnSimpleMotorFeedforward.calculate(prevTurnVelocity+turnSpeedCorrectionDps, (state.velocity-prevTurnVelocity) / period);
        if (!turnPIDController.atGoal()) {
            turn.setVoltage(MathUtil.clamp(turnVolts, -12.0, 12.0));
        } else {
            turn.setVoltage(turnSimpleMotorFeedforward.calculate(goal.velocity));
        }        
        prevTurnVelocity = state.velocity;
        SmartDashboard.putNumber(moduleString + " error (deg)", turnPIDController.getPositionError());
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
            angle = MathUtil.inputModulus(angle, -180, 180);
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
        SmartDashboard.putNumber("AntiGravitational accelration", antiGravitationalA);
        return antiGravitationalA;
    }
    /**
     * Sets the speed for the drive motor controller.
     * @param speedMps     The desired speed in meters per second.
     */
    private void setSpeed(double speedMps) {
        // Compute desired and actual speeds in m/s
        double desiredSpeed = speedMps * driveModifier;
        double actualSpeed = getCurrentSpeed();
        SmartDashboard.putNumber(moduleString + " Desired Speed (mps)", desiredSpeed);
        SmartDashboard.putNumber(moduleString + " Actual Speed (mps)", actualSpeed);
        double targetVoltage = (actualSpeed >= 0 ? forwardSimpleMotorFF :
                                 backwardSimpleMotorFF).calculate(desiredSpeed, calculateAntiGravitationalA(pitchDegSupplier.get(), rollDegSupplier.get()));//clippedAcceleration);

        // Use robot characterization as a simple physical model to account for internal resistance, frcition, etc.
        // Add a PID adjustment for error correction (also "drives" the actual speed to the desired speed)
        targetVoltage += drivePIDController.calculate(actualSpeed, desiredSpeed);
        double appliedVoltage = MathUtil.clamp(targetVoltage, -12, 12);
        drive.setVoltage(appliedVoltage);
    }

    /**
     * Sets the angle for the turn motor controller.
     * @param angle     The desired angle, between 180 degrees clockwise and 180 degrees counterclockwise.
     */
    private void setAngle(double angle) {
        double deltaTime = timer.get();
        timer.reset();
        timer.start();
        double maxDeltaTheta = Math.asin(deltaTime*config.autoCentripetalAccel/(Math.abs(getCurrentSpeed())+0.0001));
        setMaxTurnVelocity(maxDeltaTheta*180/Math.PI);
        //SmartDashboard.putNumber(moduleString + "Target Angle:", 360 * angle * (reversed ? -1 : 1));
		
        // Find the minimum distance to travel from lastAngle to angle and determine the
        // correct direction to trvel the minimum distance. This is used in order to accurately
        // calculate the goal velocity.
        double angleDiff = MathUtil.inputModulus(angle - lastAngle, -180, 180);
        SmartDashboard.putNumber(moduleString + " angleDiff (deg)", angleDiff);

        turnPIDController.setGoal(new TrapezoidProfile.State(angle * (reversed ? -1 : 1), angleDiff * (reversed ? -1 : 1) / deltaTime));
        lastAngle = angle;
    }

    /**
     * Gets the current angle of the module
     * @return module angle in degrees
     */
    public double getModuleAngle() {
        return MathUtil.inputModulus(turnEncoder.getAbsolutePosition()-turnZero, -180, 180);
    }

    /**
     * Gets the current state (speed and angle) of this module.
     * @return A SwerveModuleState object representing the speed and angle of the module.
     */
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getCurrentSpeed(), Rotation2d.fromDegrees(getModuleAngle()));
    }

    public double getCurrentSpeed() {
        return drive.getEncoder().getVelocity();
    }

    /**
     * Updates SmartDashboard with information about this module.
     */
    public void updateSmartDashboard() {
        // Display the position of the quadrature encoder.
        SmartDashboard.putNumber(moduleString + " Incremental Position", turnEncoder.getPosition());
        // Display the position of the analog encoder.
        SmartDashboard.putNumber(moduleString + " Absolute Angle (deg)", turnEncoder.getAbsolutePosition());
        // Display the module angle as calculated using the absolute encoder.
        SmartDashboard.putNumber(moduleString + " Turn Measured Pos (deg)", getModuleAngle());
        SmartDashboard.putNumber(moduleString + " Encoder Position", drive.getEncoder().getPosition());
        // Display the speed that the robot thinks it is travelling at.
        SmartDashboard.putNumber(moduleString + " Current Speed", getCurrentSpeed());
        SmartDashboard.putNumber(moduleString + " Turn Setpoint Pos (deg)", turnPIDController.getSetpoint().position);
        SmartDashboard.putNumber(moduleString + " Turn Setpoint Vel (dps)", turnPIDController.getSetpoint().velocity);
        SmartDashboard.putNumber(moduleString + " Turn Goal Pos (deg)", turnPIDController.getGoal().position);
        SmartDashboard.putNumber(moduleString + " Turn Goal Vel (dps)", turnPIDController.getGoal().velocity);
        //SmartDashboard.putNumber("Gyro Pitch", pitchDegSupplier.get());
        //SmartDashboard.putNumber("Gyro Roll", rollDegSupplier.get());
        SmartDashboard.putNumber(moduleString + "Antigravitational Acceleration", calculateAntiGravitationalA(pitchDegSupplier.get(), rollDegSupplier.get()));
        SmartDashboard.putBoolean(moduleString + " Turn is at Goal", turnPIDController.atGoal());
    }
    
    public void toggleMode() {
        if (drive.getIdleMode() == IdleMode.kBrake && turn.getIdleMode() == IdleMode.kCoast) coast();
        else brake();
    }

    public void brake() {
        drive.setIdleMode(IdleMode.kBrake);
        turn.setIdleMode(IdleMode.kBrake);
    }

    public void coast() {
        drive.setIdleMode(IdleMode.kCoast);
        turn.setIdleMode(IdleMode.kBrake);
    }

    public void setMaxTurnVelocity(double maxVel) {
        turnConstraints = new TrapezoidProfile.Constraints(maxVel, turnConstraints.maxAcceleration);
        turnPIDController.setConstraints(turnConstraints);
    }
}
