package frc.robot.lib.sim;

import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.simulation.NotifyCallback;

public class WebotsMotorForwarder implements NotifyCallback {

    private Motor motor;

    public WebotsMotorForwarder(Robot robot, String motorName) {
        motor = robot.getMotor(motorName);
        // Make sure that the motor can rotate any number of times
        motor.setPosition(Double.POSITIVE_INFINITY);
        motor.setVelocity(0);
    }

    @Override
    public void callback(String name, HALValue value) {
        motor.setVelocity(motor.getMaxVelocity() * value.getDouble());
    }

}