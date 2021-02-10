package frc.robot.lib.sim;

import com.revrobotics.CANSparkMax;

import frc.robot.lib.Mocks;

public class MockSparkMax extends CANSparkMax {
    // Assign the CAN port to a PWM port so it works with the simulator. Not a fan of this solution though
    // CAN ports should be separate from PWM ports
    private final int port;

    public MockSparkMax(int port, MotorType type) {
        super(port, type);
        this.port = port;
    }

    public static CANSparkMax createMockSparkMax(int port, MotorType type) {
        return Mocks.createMock(CANSparkMax.class, new MockSparkMax(port, type));
    }
}