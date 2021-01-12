package frc.robot.lib.sim;

import java.util.ArrayList;
import java.util.HashMap;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import frc.robot.lib.Mocks;

public class MockSparkMax extends CANSparkMax {
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
        super(port, type);
        this.port = port;
        motor = SimDevice.create("SparkMax", port);
        speed = motor.createDouble("Motor Output", false, 0);
        encoder = Mocks.createMock(CANEncoder.class, new MockedSparkEncoder(this));
        pidController = Mocks.createMock(CANPIDController.class, new MockedCANPIDController(this));
        isInverted = false;
    }

    public static CANSparkMax createMockSparkMax(int portPWM, MotorType type) {
        return Mocks.createMock(CANSparkMax.class, new MockSparkMax(portPWM, type));
    }

    @Override
    public void set(double speed) {
        speed = (isInverted ? -1.0 : 1.0) * speed;
        this.speed.set(speed);
        if (followMap.containsKey(getDeviceId())) {
            for (SimDouble motorOutput : followMap.get(getDeviceId())) motorOutput.set(speed);
        }
    }

    @Override
    public CANError follow(ExternalFollower leader, int deviceID) {
        return follow(leader, deviceID, false);
    }

    @Override
    public CANError follow(ExternalFollower leader, int deviceID, boolean invert) {
        if (!followMap.containsKey(deviceID)) {
            ArrayList<SimDouble> arr = new ArrayList<SimDouble>();
            arr.add(speed);
            followMap.put(deviceID, arr);
        } else {
            followMap.get(deviceID).add(speed);
        }
        return CANError.kOk;
    }

    @Override
    public double get() {
        return speed.get();
    }

    @Override
    public int getDeviceId() {
        return port;
    }

    @Override
    public CANEncoder getEncoder() {
        return encoder;
    }

    @Override
    public void setInverted(boolean inverted) {
        isInverted = inverted;
    }

    @Override
    public CANError restoreFactoryDefaults() {
        return CANError.kOk;
    }

    @Override
    public CANError setIdleMode(IdleMode mode) {
        return CANError.kOk;
    }

    @Override
    public CANError enableVoltageCompensation(double nominalVoltage) {
		return CANError.kOk;
	}

	@Override
	public CANError disableVoltageCompensation() {
		return CANError.kOk;
    }
    
    @Override
    public CANError setSmartCurrentLimit(int limit) {
        return CANError.kOk;
    }

    @Override
    public CANPIDController getPIDController() {
        return pidController;
    }
}