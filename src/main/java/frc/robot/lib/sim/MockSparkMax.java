package frc.robot.lib.sim;

import java.util.HashMap;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
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
    private boolean isInverted;
    // Since we need to keep a record of all the motor's followers
    private static HashMap<Integer, SimDouble> followMap = new HashMap<>();

    public MockSparkMax(int port, MotorType type) {
        super(port, type);
        this.port = port;
        motor = SimDevice.create("SparkMax", port);
        speed = motor.createDouble("Motor Output", false, 0);
        encoder = Mocks.createMock(CANEncoder.class, new MockedSparkEncoder(this));
        isInverted = false;
    }

    public static CANSparkMax createMockSparkMax(int portPWM, MotorType type) {
        return Mocks.createMock(CANSparkMax.class, new MockSparkMax(portPWM, type));
    }

    @Override
    public void set(double speed) {
        speed = (isInverted ? -1.0 : 1.0) * speed;
        this.speed.set(speed);
        if (followMap.containsKey(port)) followMap.get(port).set(speed); 
    }

    @Override
    public CANError follow(CANSparkMax leader) {
        if (!followMap.containsValue(speed)) followMap.put(leader.getDeviceId(), speed);
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
}