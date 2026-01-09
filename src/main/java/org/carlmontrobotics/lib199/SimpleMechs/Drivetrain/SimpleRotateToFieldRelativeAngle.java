package org.carlmontrobotics.lib199.SimpleMechs.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class SimpleRotateToFieldRelativeAngle extends Command {

    public final SimpleTeleopDrive teleopDrive;
    public final SimpleDrivetrain drivetrain;

    double[] thetaPIDController = {0.1,0,0};
    double positionTolerance = 5; //degrees
    double velocityTolerance = 5; //degrees/sec

    public final PIDController rotationPID = new PIDController(thetaPIDController[0], thetaPIDController[1],
            thetaPIDController[2]);

    public SimpleRotateToFieldRelativeAngle(Rotation2d angle, SimpleDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.teleopDrive = (SimpleTeleopDrive) drivetrain.getDefaultCommand();

        rotationPID.enableContinuousInput(-180, 180);
        rotationPID.setSetpoint(MathUtil.inputModulus(angle.getDegrees(), -180, 180));
        rotationPID.setTolerance(positionTolerance, velocityTolerance);
        SendableRegistry.addChild(this, rotationPID);
        // SmartDashboard.pu

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (teleopDrive == null)
            drivetrain.drive(0, 0, rotationPID.calculate(drivetrain.getHeading()));
        else {
            double[] driverRequestedSpeeds = teleopDrive.getRequestedSpeeds();
            drivetrain.drive(driverRequestedSpeeds[0], driverRequestedSpeeds[1],
                    rotationPID.calculate(drivetrain.getHeading()));
        }
    }

    @Override
    public boolean isFinished() {
        //SmartDashboard.putBoolean("At Setpoint", rotationPID.atSetpoint());
        //SmartDashboard.putNumber("Error", rotationPID.getPositionError());
        return rotationPID.atSetpoint();
    }
}