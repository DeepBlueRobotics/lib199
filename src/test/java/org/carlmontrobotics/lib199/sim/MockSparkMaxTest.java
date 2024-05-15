package org.carlmontrobotics.lib199.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.util.stream.Stream;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import org.carlmontrobotics.lib199.Mocks;
import org.carlmontrobotics.lib199.REVLibErrorAnswer;
import org.carlmontrobotics.lib199.testUtils.SafelyClosable;
import org.carlmontrobotics.lib199.testUtils.TestRules;
import org.junit.ClassRule;
import org.junit.Rule;
import org.junit.Test;

import edu.wpi.first.hal.SimDevice;
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
        SimDeviceSim simSpark = new SimDeviceSim("SparkMax[0]");
        assertNotNull(simSpark);
        SimDeviceSim simEncoder = new SimDeviceSim(simSpark.getName() + "_RelativeEncoder");
        assertNotNull(simEncoder);
        SimDouble simPosition = simEncoder.getDouble("Position");
        assertNotNull(simPosition);
    }
}
