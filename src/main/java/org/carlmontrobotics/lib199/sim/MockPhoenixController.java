package org.carlmontrobotics.lib199.sim;

import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.DoubleConsumer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

abstract class MockPhoenixController implements AutoCloseable, DoubleConsumer {
    private final int portPWM;
    private boolean isInverted;
    // Assign the CAN port to a PWM port so it works with the simulator. Not a fan of this solution though
    // CAN ports should be separate from PWM ports
    protected PWMMotorController motorPWM;
    // Since we need to keep a record of all the motor's followers
    protected static ConcurrentHashMap<Integer, CopyOnWriteArrayList<DoubleConsumer>> followMap = new ConcurrentHashMap<>();

    public MockPhoenixController(int portPWM) {
        this.portPWM = portPWM;
        isInverted = false;
    }

    @Override
    public void accept(double value) {
        set(value);
    }

    public void set(double speed) {
        motorPWM.set((getInverted() ? -1.0 : 1.0) * speed);
        if (followMap.containsKey(getDeviceID())) {
            // For CTRE controllers, the follower receives the pre-leader-inversion speed and depends on the inversion state from setInverted
            // For following inversion semantics see the "Motor Inversion Testing Results" section of the "Programming Resources/Documentation" document in the the team drive
            for (DoubleConsumer motorOutputSetter : followMap.get(getDeviceID())) motorOutputSetter.accept(speed);
        }
    }

    public double get() {
        return motorPWM.get();
    }

    public void follow(IMotorController leader) {
        // For CTRE controllers, the follower receives the pre-leader-inversion speed and depends on the inversion state from setInverted
        // For following inversion semantics see the "Motor Inversion Testing Results" section of the "Programming Resources/Documentation" document in the the team drive
        if (!followMap.containsKey(leader.getDeviceID())) {
            followMap.put(leader.getDeviceID(), new CopyOnWriteArrayList<>());
        }
        followMap.values().forEach(followerList -> followerList.remove(this));
        followMap.get(leader.getDeviceID()).add(this);
    }

    public void setInverted(boolean invert) {
        isInverted = invert;
    }

    public boolean getInverted() {
        return isInverted;
	}

	public int getDeviceID() { return portPWM; }

    public ControlMode getControlMode() { return ControlMode.PercentOutput; }

    @Override
    public void close() {
        motorPWM.close();
        followMap.values().forEach(followerList -> followerList.remove(this));
    }

    public void disable() {
        set(0);
    }

    public void setVoltage(double outputVolts) {
        set(outputVolts / 12.0);
    }
}
