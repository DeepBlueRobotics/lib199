package org.carlmontrobotics.lib199.sim;

import static org.junit.Assert.assertNotNull;

import com.revrobotics.CANSparkLowLevel.MotorType;

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
        SimDeviceSim simSpark = new SimDeviceSim("SparkMax", 0);
        assertNotNull(simSpark);
        SimDeviceSim simEncoder = new SimDeviceSim(simSpark.getName() + "_RelativeEncoder");
        assertNotNull(simEncoder);
        SimDouble simPosition = simEncoder.getDouble("Position");
        assertNotNull(simPosition);
    }
}
