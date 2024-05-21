package org.carlmontrobotics.lib199.sim;

import org.carlmontrobotics.lib199.DummySparkMaxAnswer;
import org.carlmontrobotics.lib199.Mocks;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class MockSparkMax extends MockSparkBase {

    public MockSparkMax(int port, MotorType type) {
        super(port, type, "CANSparkMax", 42);
    }

    public static CANSparkMax createMockSparkMax(int portPWM, MotorType type) {
        return Mocks.createMock(CANSparkMax.class, new MockSparkMax(portPWM, type), new DummySparkMaxAnswer());
    }
}
