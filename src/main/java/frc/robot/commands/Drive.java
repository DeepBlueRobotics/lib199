package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {
    private final Drivetrain drivetrain;
    private final Joystick joystick;
    private final double JOY_THRESHOLD = 0.08;
    
    public Drive(Drivetrain drivetrain, Joystick joystick) {
        addRequirements(this.drivetrain = drivetrain);
        this.joystick = joystick;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double speed = joystick.getRawAxis(1);
        double rotation = -joystick.getRawAxis(2);
        
        speed = Math.copySign(speed * speed, speed);
        rotation = Math.copySign(rotation * rotation, rotation);
        if (Math.abs(speed) < JOY_THRESHOLD) speed = 0.0;
        if (Math.abs(rotation) < JOY_THRESHOLD) rotation = 0.0;

        drivetrain.arcadeDrive(speed, rotation);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.arcadeDrive(0, 0);
    }
}