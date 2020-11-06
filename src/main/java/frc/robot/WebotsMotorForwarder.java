package frc.robot;

import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.simulation.NotifyCallback;

public class WebotsMotorForwarder implements NotifyCallback {

    private Motor motor;
    private double motorConstant;

    public WebotsMotorForwarder(Robot robot, String motorName, double motorConstant) {
        motor = robot.getMotor(motorName);
        // Make sure that the motor can rotate any number of times
        motor.setPosition(Double.POSITIVE_INFINITY);
        motor.setVelocity(0);
        this.motorConstant = motorConstant;
    }

    @Override
    public void callback(String name, HALValue value) {
        motor.setVelocity(motorConstant * value.getDouble());
    }

}