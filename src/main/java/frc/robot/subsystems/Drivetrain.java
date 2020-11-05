package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.cyberbotics.webots.controller.Robot;

import edu.wpi.first.hal.sim.CallbackStore;
import edu.wpi.first.hal.sim.PWMSim;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.WebotsMotorForwarder;
import frc.robot.lib.MockTalonSRX;

public class Drivetrain extends SubsystemBase {
    public enum Side {
        LEFT, RIGHT;
    }

    private final WPI_TalonSRX leftMaster, rightMaster, leftSlave, rightSlave;
    private final DifferentialDrive differentialDrive;

    private CallbackStore stores[] = new CallbackStore[4];

    public Drivetrain(Robot robot) {
        if (RobotBase.isSimulation()) {
            leftMaster = MockTalonSRX.createMockTalonSRX(Constants.CANPorts.dtFrontLeft);
            rightMaster = MockTalonSRX.createMockTalonSRX(Constants.CANPorts.dtFrontRight);
            leftSlave = MockTalonSRX.createMockTalonSRX(Constants.CANPorts.dtBackLeft);
            rightSlave = MockTalonSRX.createMockTalonSRX(Constants.CANPorts.dtBackRight);

            // For each motor controller, register a "speed callback" which calls WebotsMotorForwarder.callback() everytime the speed is set
            stores[0] = new PWMSim(Constants.CANPorts.dtFrontLeft).registerSpeedCallback(new WebotsMotorForwarder(robot, "Front Left", Constants.neoMotorConstant), true);
            stores[1] = new PWMSim(Constants.CANPorts.dtFrontRight).registerSpeedCallback(new WebotsMotorForwarder(robot, "Front Right", Constants.neoMotorConstant), true);
            stores[2] = new PWMSim(Constants.CANPorts.dtBackLeft).registerSpeedCallback(new WebotsMotorForwarder(robot, "Back Left", Constants.neoMotorConstant), true);
            stores[3] = new PWMSim(Constants.CANPorts.dtBackRight).registerSpeedCallback(new WebotsMotorForwarder(robot, "Back Right", Constants.neoMotorConstant), true);
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