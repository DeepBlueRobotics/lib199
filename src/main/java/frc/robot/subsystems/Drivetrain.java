package frc.robot.subsystems;

import com.cyberbotics.webots.controller.Gyro;
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.Robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.sim.SimDeviceSim;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.lib.sim.Field2d;
import frc.robot.lib.sim.MockSparkMax;

public class Drivetrain extends SubsystemBase {
    public enum Side {
        LEFT, RIGHT;
    }

    private final CANSparkMax leftMaster, rightMaster, leftSlave, rightSlave;
    private final CANEncoder leftEnc, rightEnc;

    private final AHRS gyro = new AHRS(Port.kUSB1);
    private final boolean gyroReversed = false;

    private final DifferentialDrive differentialDrive;
    private final DifferentialDriveKinematics kinematics;
    private DifferentialDriveOdometry odometry;

    private PositionSensor leftEncSim;
    private PositionSensor rightEncSim;
    private Gyro gyroSim;

    private SimDeviceSim wpiGyroSim;
    private double angle;
    private double timestep;
    // Field2d is a class defined by WPIlib that allows the user to see the robot pose in the simulator
    // For some reason it was not included in the edu.wpi.first.simulation package so I needed to add it manually.
    // TODO: This will be fixed in a future WPIlib version, so make sure to get rid of frc.robot.lib.Field2d.
    private final Field2d field = new Field2d();

    public Drivetrain(Robot robot) {
        if (RobotBase.isSimulation()) {
            // Use the mockito code to create mocked/overriden versions of CANSparkMax objects
            leftMaster = MockSparkMax.createMockSparkMax(Constants.CANPorts.dtFrontLeft,
                    CANSparkMaxLowLevel.MotorType.kBrushless);
            rightMaster = MockSparkMax.createMockSparkMax(Constants.CANPorts.dtFrontRight,
                    CANSparkMaxLowLevel.MotorType.kBrushless);
            leftSlave = MockSparkMax.createMockSparkMax(Constants.CANPorts.dtBackLeft,
                    CANSparkMaxLowLevel.MotorType.kBrushless);
            rightSlave = MockSparkMax.createMockSparkMax(Constants.CANPorts.dtBackRight,
                    CANSparkMaxLowLevel.MotorType.kBrushless);

            // Calling the getEncoder() method on a mocked CANSparkMax will return a mocked CANEncoder
            leftEnc = leftMaster.getEncoder();
            rightEnc = rightMaster.getEncoder();

            // Convert from seconds to miliseconds
            timestep = robot.getBasicTimeStep() / 1000;
            
            // Get the Webots encoders and enable them
            leftEncSim = robot.getPositionSensor("Front Left Encoder");
            rightEncSim = robot.getPositionSensor("Front Right Encoder");
            leftEncSim.enable(20);
            rightEncSim.enable(20);

            // Get the Webots gyro and enable it
            gyroSim = robot.getGyro("gyro");
            gyroSim.enable(20);

            // Get the simulated nav-X in the WPIlib simulator
            wpiGyroSim = new SimDeviceSim("navX-Sensor[0]");

            // For each motor controller, register a "speed callback" which calls WebotsMotorForwarder.callback() everytime the speed is set
            // new PWMSim(Constants.CANPorts.dtFrontLeft).registerSpeedCallback(new WebotsMotorForwarder(robot, "Front Left", Constants.neoMotorConstant), true);
            // new PWMSim(Constants.CANPorts.dtFrontRight).registerSpeedCallback(new WebotsMotorForwarder(robot, "Front Right", Constants.neoMotorConstant), true);
            // new PWMSim(Constants.CANPorts.dtBackLeft).registerSpeedCallback(new WebotsMotorForwarder(robot, "Back Left", Constants.neoMotorConstant), true);
            // new PWMSim(Constants.CANPorts.dtBackRight).registerSpeedCallback(new WebotsMotorForwarder(robot, "Back Right", Constants.neoMotorConstant), true);
        } 
        // If not simulated, go about the usual business
        else {
            leftMaster = new CANSparkMax(Constants.CANPorts.dtFrontLeft, CANSparkMaxLowLevel.MotorType.kBrushless);
            rightMaster = new CANSparkMax(Constants.CANPorts.dtFrontRight, CANSparkMaxLowLevel.MotorType.kBrushless);
            leftSlave = new CANSparkMax(Constants.CANPorts.dtBackLeft, CANSparkMaxLowLevel.MotorType.kBrushless);
            rightSlave = new CANSparkMax(Constants.CANPorts.dtBackRight, CANSparkMaxLowLevel.MotorType.kBrushless);

            leftEnc = leftMaster.getEncoder();
            rightEnc = rightMaster.getEncoder();
        }
        differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
        
        rightMaster.setInverted(true);

        gyro.reset();
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
        kinematics = new DifferentialDriveKinematics(Constants.trackWidth);

        // conversion is in m/rev
        double conversion = Constants.wheelDiameter * Math.PI / Constants.motorGearing;
        leftEnc.setPositionConversionFactor(conversion);
        leftEnc.setVelocityConversionFactor(conversion / 60.);
        rightEnc.setPositionConversionFactor(conversion);
        rightEnc.setVelocityConversionFactor(conversion / 60.);
    }

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            // Get the position of the Webots encoders and set the position of the WPIlib encoders 
            // getValue() returns radians
            leftEnc.setPosition(leftEncSim.getValue() * Constants.wheelDiameter);
            rightEnc.setPosition(rightEncSim.getValue() * Constants.wheelDiameter);

            /* getValues() returns angular speeds about each axis (x, y, z).
               reading represents the change in angular position about the y axis.
               getValues()[1] is negated to convert from Webot's coordinate system (counter-clockwise = positive) to WPIlib's coordinate system (counter-clockwise = negative).
            */
            double reading = -gyroSim.getValues()[1] * timestep;
            // In testing, reading was sometimes NAN in the first second of the simulation.
            // Also convert from radians to degrees
            angle += Double.isNaN(reading) ? 0 : (180 * reading / Math.PI);
            // Make sure angle is between 0 and 359 inclusive
            angle = Math.copySign(Math.abs(angle) % 360, angle);
            // Update the WPIlib gyro
            wpiGyroSim.getValue("Yaw").setValue(HALValue.makeDouble(angle));
        }
        odometry.update(Rotation2d.fromDegrees(getHeading()), getEncoderPosition(Side.LEFT), getEncoderPosition(Side.RIGHT));
        field.setRobotPose(odometry.getPoseMeters());
    }

    public void loadOdometry(Pose2d pose, double heading) {
        angle = 180 * heading / Math.PI;
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(heading), pose);
    }

    public double getEncoderPosition(Side side) {
        if (side == Side.LEFT) {
            return leftEnc.getPosition();
        } else {
            return rightEnc.getPosition();
        }
    }
    
    public void arcadeDrive(double speed, double rotation) {
        differentialDrive.arcadeDrive(speed, rotation, false);
    }

    public void tankDrive(double left, double right) {
        differentialDrive.tankDrive(left, right, false);
    }

    // tankDriveDirect is for autonomous path following
    public void tankDriveDirect(double left, double right) {
        differentialDrive.tankDrive(left / Constants.maxSpeed, right / Constants.maxSpeed, false);
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360) * (gyroReversed ? -1.0 : 1.0);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveOdometry getOdometry() {
        return odometry;
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }
}