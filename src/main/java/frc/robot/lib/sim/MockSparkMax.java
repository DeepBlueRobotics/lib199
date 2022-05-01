package frc.robot.lib.sim;

import java.util.ArrayList;
import java.util.HashMap;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import frc.robot.lib.REVLibErrorAnswer;
import frc.robot.lib.DummySparkMaxAnswer;
import frc.robot.lib.Mocks;

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
    // Since we need to keep a record of all the motor's followers
    private static HashMap<Integer, ArrayList<SimDouble>> followMap = new HashMap<>();

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
            for (SimDouble motorOutput : followMap.get(getDeviceId())) motorOutput.set(speed);
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
        if (!followMap.containsKey(deviceID)) {
            followMap.put(deviceID, new ArrayList<SimDouble>());
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