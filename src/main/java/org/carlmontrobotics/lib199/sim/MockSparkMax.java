package org.carlmontrobotics.lib199.sim;

import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.DoubleConsumer;

import org.carlmontrobotics.lib199.DummySparkMaxAnswer;
import org.carlmontrobotics.lib199.Mocks;
import org.carlmontrobotics.lib199.REVLibErrorAnswer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;

public class MockSparkMax {
    // Assign the CAN port to a PWM port so it works with the simulator. Not a fan
    // of this solution though
    // CAN ports should be separate from PWM ports
    private final int port;
    private final SimDevice motor;
    private final SimDouble speed;
    private RelativeEncoder encoder;
    private SparkMaxPIDController pidController;
    private boolean isInverted;
    // We need to store the function so we can remove it from the follower list when this motor is no longer a follower
    private DoubleConsumer followFunction;
    // Since we need to keep a record of all the motor's followers
    private static ConcurrentHashMap<Integer, CopyOnWriteArrayList<DoubleConsumer>> followMap = new ConcurrentHashMap<>();

    public MockSparkMax(int port, MotorType type) {
        this.port = port;
        motor = SimDevice.create("SparkMax", port);
        speed = motor.createDouble("Motor Output", Direction.kOutput, 0);
        encoder = Mocks.createMock(RelativeEncoder.class, new MockedSparkEncoder(port), new REVLibErrorAnswer());
        pidController = Mocks.createMock(SparkMaxPIDController.class, new MockedSparkMaxPIDController(), new REVLibErrorAnswer());
        isInverted = false;
    }

    public static CANSparkMax createMockSparkMax(int portPWM, MotorType type) {
        return Mocks.createMock(CANSparkMax.class, new MockSparkMax(portPWM, type), new DummySparkMaxAnswer());
    }

    public void set(double speed) {
        speed = (isInverted ? -1.0 : 1.0) * speed;
        this.speed.set(speed);
        if (followMap.containsKey(getDeviceId())) {
            // For spark maxes, the follower receives the post-leader-inversion speed and does not depend on the inversion state from setInverted
            // For following inversion semantics see the "Motor Inversion Testing Results" section of the "Programming Resources/Documentation" document in the the team drive
            for (DoubleConsumer motorOutputSetter : followMap.get(getDeviceId())) motorOutputSetter.accept(speed);
        }
    }

    public REVLibError follow(CANSparkMax leader) {
        return follow(leader, false);
    }

    public REVLibError follow(CANSparkMax leader, boolean invert) {
		return follow(ExternalFollower.kFollowerSparkMax, leader.getDeviceId(), invert);
	}

    public REVLibError follow(ExternalFollower leader, int deviceID) {
        return follow(leader, deviceID, false);
    }

    public REVLibError follow(ExternalFollower leader, int deviceID, boolean invert) {
        // For spark maxes, the follower receives the post-leader-inversion speed and does not depend on the inversion state from setInverted
        // For following inversion semantics see the "Motor Inversion Testing Results" section of the "Programming Resources/Documentation" document in the the team drive
        if (!followMap.containsKey(deviceID)) {
            followMap.put(deviceID, new CopyOnWriteArrayList<>());
        }
        if(followFunction != null) {
            followMap.values().forEach(followList -> followList.remove(followFunction));
        }
        double inversionMultiplier = (invert ? -1.0 : 1.0);
        followMap.get(deviceID).add(newSpeed -> speed.set(inversionMultiplier * newSpeed));
        return REVLibError.kOk;
    }

    public double get() {
        return speed.get();
    }

    public int getDeviceId() {
        return port;
    }

    public RelativeEncoder getEncoder() {
        return encoder;
    }

    public void setInverted(boolean inverted) {
        isInverted = inverted;
    }

    public REVLibError restoreFactoryDefaults() {
        return REVLibError.kOk;
    }

    public REVLibError setIdleMode(IdleMode mode) {
        return REVLibError.kOk;
    }

    public REVLibError enableVoltageCompensation(double nominalVoltage) {
		return REVLibError.kOk;
	}

	public REVLibError disableVoltageCompensation() {
		return REVLibError.kOk;
    }

    public REVLibError setSmartCurrentLimit(int limit) {
        return REVLibError.kOk;
    }

    public SparkMaxPIDController getPIDController() {
        return pidController;
    }

    public void setVoltage(double outputVolts) {
        set(outputVolts / 12);
    }
}