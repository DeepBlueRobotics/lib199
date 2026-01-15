package org.carlmontrobotics.lib199.sim;

import org.carlmontrobotics.lib199.Mocks;
import org.carlmontrobotics.lib199.REVLibErrorAnswer;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;

public class MockSparkFlex extends MockSparkBase {

    public MockSparkFlex(int port, MotorType type) {
        super(port, type, "CANSparkFlex", 7168, NEOType.VORTEX);
    }

    public static SparkFlex createMockSparkFlex(int portPWM, MotorType type) {
        return Mocks.createMock(SparkFlex.class, new MockSparkFlex(portPWM, type), new REVLibErrorAnswer());
    }
}
