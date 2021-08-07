package frc.robot.lib.sim;

import java.util.ArrayList;
import java.util.HashMap;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import frc.robot.lib.CANErrorAnswer;
import frc.robot.lib.DummySparkMaxAnswer;
import frc.robot.lib.Mocks;

public class MockSparkMax {
    // Assign the CAN port to a PWM port so it works with the simulator. Not a fan of this solution though
    // CAN ports should be separate from PWM ports
    private final int port;
    private final SimDevice motor;
    private final SimDouble speed;
    private CANEncoder encoder;
    private CANPIDController pidController;
    private boolean isInverted;
    // Since we need to keep a record of all the motor's followers
    private static HashMap<Integer, ArrayList<SimDouble>> followMap = new HashMap<>();

    public MockSparkMax(int port, MotorType type) {
        this.port = port;
        motor = SimDevice.create("SparkMax", port);
        speed = motor.createDouble("Motor Output", Direction.kOutput, 0);
        encoder = Mocks.createMock(CANEncoder.class, new MockedSparkEncoder(port), new CANErrorAnswer());
        pidController = Mocks.createMock(CANPIDController.class, new MockedCANPIDController(), new CANErrorAnswer());
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

    public CANError follow(CANSparkMax leader) {
        return follow(leader, false);
    }

    public CANError follow(CANSparkMax leader, boolean invert) {
		return follow(ExternalFollower.kFollowerSparkMax, leader.getDeviceId(), invert);
	}
    
    public CANError follow(ExternalFollower leader, int deviceID) {
        return follow(leader, deviceID, false);
    }

    public CANError follow(ExternalFollower leader, int deviceID, boolean invert) {
        if (!followMap.containsKey(deviceID)) {
            followMap.put(deviceID, new ArrayList<SimDouble>());
        }
        followMap.get(deviceID).add(speed);
        return CANError.kOk;
    }
    
    public double get() {
        return speed.get();
    }
    
    public int getDeviceId() {
        return port;
    }

    public CANEncoder getEncoder() {
        return encoder;
    }

    public void setInverted(boolean inverted) {
        isInverted = inverted;
    }

    public CANError restoreFactoryDefaults() {
        return CANError.kOk;
    }

    public CANError setIdleMode(IdleMode mode) {
        return CANError.kOk;
    }

    public CANError enableVoltageCompensation(double nominalVoltage) {
		return CANError.kOk;
	}

	public CANError disableVoltageCompensation() {
		return CANError.kOk;
    }
    
    public CANError setSmartCurrentLimit(int limit) {
        return CANError.kOk;
    }

    public CANPIDController getPIDController() {
        return pidController;
    }
}