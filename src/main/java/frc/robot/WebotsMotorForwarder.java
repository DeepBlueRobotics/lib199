package frc.robot;

import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.sim.NotifyCallback;

public class WebotsMotorForwarder implements NotifyCallback {

    private Motor motor;
    private double motorConstant;

    public WebotsMotorForwarder(Robot robot, String motorName, double motorConstant) {
        motor = robot.getMotor(motorName);
        // Make sure that the motor can rotate any number of times
        motor.setPosition(Double.POSITIVE_INFINITY);
        motor.setVelocity(0);
        this.motorConstant = motorConstant;
        System.out.println("WebotsMotorForwarder created for motor " + motor.getName());
    }

    @Override
    public void callback(String name, HALValue value) {
        motor.setVelocity(motorConstant * value.getDouble());
        System.out.println("called setVelocity(" + motorConstant * value.getDouble() + ") for motor " + motor.getName());
    }

}