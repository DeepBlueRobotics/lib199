package frc.robot.lib.sim;

import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.simulation.NotifyCallback;

public class WebotsMotorForwarder implements NotifyCallback, Runnable {

    private double currentOutput;
    private Motor motor;

    public WebotsMotorForwarder(Robot robot, String motorName) {
        motor = robot.getMotor(motorName);
        currentOutput = 0;
        // Make sure that the motor can rotate any number of times
        motor.setPosition(Double.POSITIVE_INFINITY);
        motor.setVelocity(0);
        Simulation.registerPeriodicMethod(this);
    }

    @Override
    public void callback(String name, HALValue value) {
        currentOutput = value.getDouble();
    }

    @Override
    public void run() {
        motor.setVelocity(motor.getMaxVelocity() * currentOutput);
    }

}