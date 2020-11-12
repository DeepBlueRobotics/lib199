package frc.robot.lib.sim;

import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.simulation.NotifyCallback;

/**
 * Forwards motor calls from WPILib motor controllers to Webots
 */
public class WebotsMotorForwarder implements NotifyCallback, Runnable {

    private double currentOutput;
    private Motor motor;

    /**
     * Creates a new WebotsMotorForwarder
     * @param robot the Webots robot
     * @param motorName the name of the Webots motor to which to connect
     */
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