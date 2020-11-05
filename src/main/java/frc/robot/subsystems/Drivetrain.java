package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.cyberbotics.webots.controller.Robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.lib.sim.MockTalonSRX;

public class Drivetrain extends SubsystemBase {
    public enum Side {
        LEFT, RIGHT;
    }

    private final WPI_TalonSRX leftMaster, rightMaster, leftSlave, rightSlave;
    private final DifferentialDrive differentialDrive;

    public Drivetrain(Robot robot) {
        if (RobotBase.isSimulation()) {
            leftMaster = MockTalonSRX.createMockTalonSRX(Constants.CANPorts.dtFrontLeft);
            rightMaster = MockTalonSRX.createMockTalonSRX(Constants.CANPorts.dtFrontRight);
            leftSlave = MockTalonSRX.createMockTalonSRX(Constants.CANPorts.dtBackLeft);
            rightSlave = MockTalonSRX.createMockTalonSRX(Constants.CANPorts.dtBackRight);
        } 
        // If not simulated, go about the usual business
        else {
            leftMaster = new WPI_TalonSRX(Constants.CANPorts.dtFrontLeft);
            rightMaster = new WPI_TalonSRX(Constants.CANPorts.dtFrontRight);
            leftSlave = new WPI_TalonSRX(Constants.CANPorts.dtBackLeft);
            rightSlave = new WPI_TalonSRX(Constants.CANPorts.dtBackRight);
        }
        differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
        
        rightMaster.setInverted(true);
    }

    @Override
    public void periodic() {
    }
    
    public void arcadeDrive(double speed, double rotation) {
        differentialDrive.arcadeDrive(speed, rotation, false);
    }

    public void tankDrive(double left, double right) {
        differentialDrive.tankDrive(left, right, false);
    }
}