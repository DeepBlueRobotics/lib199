package org.carlmontrobotics.lib199.sim;

import org.carlmontrobotics.lib199.testUtils.TestRules;
import org.junit.ClassRule;
import org.junit.Rule;
import org.junit.Test;

import static org.hamcrest.CoreMatchers.*;
import static org.hamcrest.MatcherAssert.*;
import static org.junit.Assert.assertEquals;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;

import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class MockedCANCoderTest {
    @ClassRule
    public static TestRules.InitializeHAL simClassRule = new TestRules.InitializeHAL(); 
    @Rule
    public TestRules.ResetSimDeviceSimData simTestRule = new TestRules.ResetSimDeviceSimData(); 

    @Test
    public void testCountUpdatesPosition() throws InterruptedException {
        CANcoder canCoder = new CANcoder(0);

        // Set up a SimDevice for the CANcoder
        new MockedCANCoder(canCoder);

        // Consider the current position to be 0 rotations.
        canCoder.setPosition(0.0);

        // Set the position to 0.42 rotations via the SimDevice interface
        SimDeviceSim canCoderSim = new SimDeviceSim("CANCoder", 0);
        canCoderSim.getDouble("count").set(0.42 * MockedCANCoder.kCANCoderCPR);

        // Wait for the CANCoder to update the position. This appears to happen on a separate thread.
        canCoder.getPosition().waitForUpdate(1.0);

        assertEquals(0.42, canCoder.getPosition().getValueAsDouble(), 0.001);
    }

}
