package org.carlmontrobotics.lib199.sim;

import org.carlmontrobotics.lib199.Mocks;
import org.carlmontrobotics.lib199.REVLibErrorAnswer;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class MockSparkMax extends MockSparkBase {

    public MockSparkMax(int port, MotorType type, NEOType neoType) {
        super(port, type, "CANSparkMax", 42, neoType);
    }

    public static SparkMax createMockSparkMax(int portPWM, MotorType type, NEOType neoType) {
        return Mocks.createMock(SparkMax.class, new MockSparkMax(portPWM, type, neoType), new REVLibErrorAnswer());
    }
}
