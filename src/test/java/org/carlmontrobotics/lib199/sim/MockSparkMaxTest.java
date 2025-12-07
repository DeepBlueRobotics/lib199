package org.carlmontrobotics.lib199.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.carlmontrobotics.lib199.testUtils.TestRules;
import org.junit.ClassRule;
import org.junit.Rule;
import org.junit.Test;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class MockSparkMaxTest {
    @ClassRule
    public static TestRules.InitializeHAL simClassRule = new TestRules.InitializeHAL();
    @Rule
    public TestRules.ResetSimDeviceSimData simTestRule = new TestRules.ResetSimDeviceSimData();

    @Test
    public void testHasEncoder() {
        var mockSpark = new MockSparkMax(0, MotorType.kBrushless);
        SimDeviceSim simSpark = new SimDeviceSim("CANMotor:CANSparkMax", 0);
        assertNotNull(simSpark);
        SimDeviceSim simEncoder = new SimDeviceSim("CANEncoder:CANSparkMax", 0);
        assertNotNull(simEncoder);
        SimDouble simPosition = simEncoder.getDouble("position");
        assertNotNull(simPosition);
    }

    @Test
    public void testCanRecreateIfClosed() {
        for (int i = 0; i < 2; i++) {
            try (var mockSpark = new MockSparkMax(0, MotorType.kBrushless)) {
                SimDeviceSim simSpark =
                        new SimDeviceSim("CANMotor:CANSparkMax", 0);
                assertNotNull(simSpark);
                SimDeviceSim simEncoder =
                        new SimDeviceSim("CANEncoder:CANSparkMax", 0);
                assertNotNull(simEncoder);
                SimDouble simPosition = simEncoder.getDouble("position");
                assertNotNull(simPosition);
            }
        }
    }

    @Test
    public void testGetOutputCurrent() {
        var mockSpark = new MockSparkMax(0, MotorType.kBrushless);
        SimDeviceSim simSpark = new SimDeviceSim("CANMotor:CANSparkMax", 0);
        assertNotNull(simSpark);
        SimDouble simCurrent = simSpark.getDouble("motorCurrent");
        assertNotNull(simCurrent);
        simCurrent.set(42.0);
        assertEquals(42, mockSpark.getOutputCurrent(), 1e-6);
    }
}
