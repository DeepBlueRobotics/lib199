package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.lib.MotorControllerFactory;

public class Drivetrain extends SubsystemBase {
    public enum Side {
        LEFT, RIGHT;
    }

    private final CANSparkMax leftMaster, rightMaster, leftSlave, rightSlave;
    private final CANEncoder leftEnc, rightEnc;
    private final DifferentialDrive differentialDrive;
    private final DifferentialDriveKinematics kinematics;
    private DifferentialDriveOdometry odometry;

    // Field2d is a class defined by WPIlib that allows the user to see the robot pose in the simulator
    // For some reason it was not included in the edu.wpi.first.simulation package so I needed to add it manually.
    // TODO: This will be fixed in a future WPIlib version, so make sure to get rid of frc.robot.lib.Field2d.
    private final Field2d field = new Field2d();

    private final AHRS gyro = new AHRS(Port.kUSB1);
    private final boolean gyroReversed = false;

    public Drivetrain() {
        leftMaster = MotorControllerFactory.createSparkMax(Constants.CANPorts.dtFrontLeft);
        rightMaster = MotorControllerFactory.createSparkMax(Constants.CANPorts.dtFrontRight);
        leftSlave = MotorControllerFactory.createSparkMax(Constants.CANPorts.dtBackLeft);
        rightSlave = MotorControllerFactory.createSparkMax(Constants.CANPorts.dtBackRight);

        leftEnc = leftMaster.getEncoder();
        rightEnc = rightMaster.getEncoder();
        differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
        
        rightMaster.setInverted(true);

        gyro.reset();
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
        kinematics = new DifferentialDriveKinematics(Constants.trackWidth);

        double conversion = Constants.wheelDiameter * Math.PI / Constants.motorGearing;
        leftEnc.setPositionConversionFactor(conversion);
        rightEnc.setPositionConversionFactor(conversion);
    }

    @Override
    public void periodic() {
        odometry.update(Rotation2d.fromDegrees(getHeading()), getEncoderPosition(Side.LEFT), getEncoderPosition(Side.RIGHT));
        field.setRobotPose(odometry.getPoseMeters());
    }

    public void loadOdometry(Pose2d pose, double heading) {
        //angle = 180 * heading / Math.PI;
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
