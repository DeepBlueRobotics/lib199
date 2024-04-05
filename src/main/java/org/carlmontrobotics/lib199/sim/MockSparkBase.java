package org.carlmontrobotics.lib199.sim;

import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;

import org.carlmontrobotics.lib199.DummySparkMaxAnswer;
import org.carlmontrobotics.lib199.Mocks;
import org.carlmontrobotics.lib199.REVLibErrorAnswer;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ExternalFollower;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;

public class MockSparkBase {
    // Assign the CAN port to a PWM port so it works with the simulator. Not a fan
    // of this solution though
    // CAN ports should be separate from PWM ports
    private final int port;
    private final SimDevice motor;
    private final SimDouble speed;
    private RelativeEncoder encoder;
    private SparkAbsoluteEncoder absoluteEncoder;
    private SparkPIDController pidController;
    private boolean isInverted;
    // Since we need to keep a record of all the motor's followers
    private static ConcurrentHashMap<Integer, CopyOnWriteArrayList<SimDouble>> followMap = new ConcurrentHashMap<>();

    public MockSparkBase(int port, MotorType type, String simDeviceName) {
        this.port = port;
        motor = SimDevice.create(simDeviceName, port);
        speed = motor.createDouble("Motor Output", Direction.kOutput, 0);
        encoder = Mocks.createMock(RelativeEncoder.class, new MockedSparkEncoder(port), new REVLibErrorAnswer());
        absoluteEncoder = Mocks.createMock(SparkAbsoluteEncoder.class, new MockedSparkAbsoluteEncoder(port), new REVLibErrorAnswer());
        pidController = Mocks.createMock(SparkPIDController.class, new MockedSparkMaxPIDController(), new REVLibErrorAnswer());
        isInverted = false;
    }

    public void set(double speed) {
        speed = (isInverted ? -1.0 : 1.0) * speed;
        this.speed.set(speed);
        if (followMap.containsKey(getDeviceId())) {
            for (SimDouble motorOutput : followMap.get(getDeviceId())) motorOutput.set(speed);
        }
    }

    public REVLibError follow(CANSparkMax leader) {
        return follow(leader, false);
    }

    public REVLibError follow(CANSparkMax leader, boolean invert) {
		return follow(ExternalFollower.kFollowerSpark, leader.getDeviceId(), invert);
	}
    
    public REVLibError follow(ExternalFollower leader, int deviceID) {
        return follow(leader, deviceID, false);
    }

    public REVLibError follow(ExternalFollower leader, int deviceID, boolean invert) {
        if (!followMap.containsKey(deviceID)) {
            followMap.put(deviceID, new CopyOnWriteArrayList<SimDouble>());
        }
        followMap.get(deviceID).add(speed);
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

    public SparkAbsoluteEncoder getAbsoluteEncoder() {
        return absoluteEncoder;
    }

    public SparkAbsoluteEncoder getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type encoderType) {
        return absoluteEncoder;
    }

    public SparkAbsoluteEncoder getAbsoluteEncoder(SparkAbsoluteEncoder.Type encoderType) {
        return absoluteEncoder;
    }

    public void setInverted(boolean inverted) {
        isInverted = inverted;
    }

    public boolean getInverted() {
        return isInverted;
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

    public SparkPIDController getPIDController() {
        return pidController;
    }

    public void setVoltage(double outputVolts) {
        set(outputVolts / 12);
    }
}