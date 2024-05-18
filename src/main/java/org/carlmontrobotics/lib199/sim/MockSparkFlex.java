package org.carlmontrobotics.lib199.sim;

import org.carlmontrobotics.lib199.DummySparkMaxAnswer;
import org.carlmontrobotics.lib199.Mocks;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;

public class MockSparkFlex extends MockSparkBase {

    public MockSparkFlex(int port, MotorType type) {
        super(port, type, "SparkFlex");
    }

    public static CANSparkFlex createMockSparkFlex(int portPWM, MotorType type) {
        return Mocks.createMock(CANSparkFlex.class, new MockSparkFlex(portPWM, type), new DummySparkMaxAnswer());
    }
}
