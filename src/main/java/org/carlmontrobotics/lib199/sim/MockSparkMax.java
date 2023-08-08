package org.carlmontrobotics.lib199.sim;

import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.DoubleConsumer;

import org.carlmontrobotics.lib199.DummySparkMaxAnswer;
import org.carlmontrobotics.lib199.Mocks;
import org.carlmontrobotics.lib199.REVLibErrorAnswer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;

public class MockSparkMax {

    public static final double defaultNominalVoltage = 12.0;

    private final int port;
    private final SimDevice motor;
    private final SimDouble speed;
    private RelativeEncoder encoder;
    private SparkMaxPIDController pidController;
    private boolean isInverted;
    private double voltageCompensationNominalVoltage = defaultNominalVoltage;
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
        speed *= voltageCompensationNominalVoltage / defaultNominalVoltage;
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
        if(isFollower()) {
            followMap.values().forEach(followList -> followList.remove(followFunction));
        }
        double inversionMultiplier = (invert ? -1.0 : 1.0);
        // Because ExternalFollower does not implement equals, this could result in bugs if the user passes in a custom ExternalFollower object,
        // but I think that it's unlikely and users should use the builtin definitions anyway
        if(leader.equals(ExternalFollower.kFollowerSparkMax)) {
            followMap.get(deviceID).add(followFunction = (newSpeed -> speed.set(inversionMultiplier * newSpeed)));
        } else if(leader.equals(ExternalFollower.kFollowerPhoenix)) {
            MockPhoenixController.followMap.get(deviceID).add(followFunction = (newSpeed -> speed.set(inversionMultiplier * newSpeed)));
        }
        return REVLibError.kOk;
    }

    public boolean isFollower() {
        return followFunction != null;
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

    public RelativeEncoder getEncoder(SparkMaxRelativeEncoder.Type type, int countsPerRev) {
        return getEncoder();
    }

    public void setInverted(boolean inverted) {
        isInverted = inverted;
    }

    public boolean getInverted() {
        return isInverted;
    }

    public REVLibError enableVoltageCompensation(double nominalVoltage) {
        voltageCompensationNominalVoltage = nominalVoltage;
		return REVLibError.kOk;
	}

	public REVLibError disableVoltageCompensation() {
        voltageCompensationNominalVoltage = defaultNominalVoltage;
		return REVLibError.kOk;
    }

    public double getVoltageCompensationNominalVoltage() {
        return voltageCompensationNominalVoltage;
    }

    public SparkMaxPIDController getPIDController() {
        return pidController;
    }

    public void setVoltage(double outputVolts) {
        set(outputVolts / 12);
    }

    public void disable() {
        set(0);
    }

    public double getAppliedOutput() {
        return get();
    }

    public double getBusVoltage() {
        return defaultNominalVoltage;
    }
}
