package org.carlmontrobotics.lib199;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mockingDetails; 

import org.carlmontrobotics.lib199.testUtils.ErrStreamTest;
import org.carlmontrobotics.lib199.testUtils.TestRules;
import org.junit.ClassRule;
import org.junit.Rule;
import org.junit.Test;

import com.playingwithfusion.TimeOfFlight;

public class SensorFactoryTest extends ErrStreamTest {

    @ClassRule
    public static TestRules.InitializeHAL classRule = new TestRules.InitializeHAL(); 
    @Rule
    public TestRules.ResetSimDeviceSimData testRule = new TestRules.ResetSimDeviceSimData(); 

    @Test
    // AutoClosable.close() throws Exception
    public void testCreateNoErrors() throws Exception {
        // Call close to free PWM ports
        ((AutoCloseable)SensorFactory.createCANCoder(0)).close();
        ((AutoCloseable)SensorFactory.createCANCoder(1)).close();
        ((AutoCloseable)SensorFactory.createPlayingWithFusionTimeOfFlight(0)).close();
        ((AutoCloseable)SensorFactory.createPlayingWithFusionTimeOfFlight(1)).close();
        assertEquals(0, errStream.toByteArray().length);
    }

    @Test
    public void testPlayingWithFusionTimeOfFlightIsMock() {
        // Call close to free PWM ports
        try (TimeOfFlight dev = SensorFactory.createPlayingWithFusionTimeOfFlight(0)) {
            assertTrue(mockingDetails(dev).isMock());
        }
    }
}
