package org.carlmontrobotics.lib199.sim;

import org.carlmontrobotics.lib199.DummySparkMaxAnswer;
import org.carlmontrobotics.lib199.Mocks;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class MockSparkMax extends MockSparkBase {

    public MockSparkMax(int port, MotorType type) {
        super(port, type, "CANSparkMax", 42);
    }

    public static SparkMax createMockSparkMax(int portPWM, MotorType type) {
        return Mocks.createMock(SparkMax.class, new MockSparkMax(portPWM, type), new DummySparkMaxAnswer());
    }
}
