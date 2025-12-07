package org.carlmontrobotics.lib199.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.util.stream.Stream;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

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

public class MockedEncoderTest {

    @ClassRule
    public static TestRules.InitializeHAL simClassRule = new TestRules.InitializeHAL(); 
    @Rule
    public TestRules.ResetSimDeviceSimData simTestRule = new TestRules.ResetSimDeviceSimData(); 

    @Test
    public void testDeviceCreation() {
        assertTestDeviceCreation(0);
        assertTestDeviceCreation(1);
        assertTestDeviceCreation(2);
    }

    private void assertTestDeviceCreation(int id) {
        String deviceName = String.format("testDevice[%d]", id);
        assertFalse(simDeviceExists(deviceName));
        try(SafelyClosable closableEncoder = createEncoder(id)) {
            assertTrue(simDeviceExists(deviceName));
            SimDeviceSim sim = new SimDeviceSim("testDevice", id);
            assertEquals(1, Stream.of(sim.enumerateValues())
                .map(info -> info.name)
                .distinct()
                .filter(name -> name.equals("position")).count());
        }
        assertFalse(simDeviceExists(deviceName));
    }

    @Test
    public void testFunctionality() {
        withEncoders((enc, sim, positionSim) -> {
            testFunctionalityWithPositionConversionFactor(1, enc, positionSim);
            testFunctionalityWithPositionConversionFactor(10, enc, positionSim);
            testFunctionalityWithPositionConversionFactor(100, enc, positionSim);
        });
    }

    private void testFunctionalityWithPositionConversionFactor(double factor, RelativeEncoder enc, SimDouble positionSim) {
        assertEquals(REVLibError.kOk, enc.setPositionConversionFactor(factor)); //FIXME: make this in a config with positionConversionFactor
        assertEquals(factor, enc.getPositionConversionFactor(), 0.01);
        testPosition(10, enc, factor, positionSim);
        testPosition(0, enc, factor, positionSim);
        testPosition(-10, enc, factor, positionSim);
    }

    private void testPosition(double position, RelativeEncoder enc, double conversionFactor, SimDouble positionSim) {
        // This test fails with a delta of 0.01
        assertEquals(REVLibError.kOk, enc.setPosition(position));
        assertEquals(position, enc.getPosition(), 0.02);
        assertEquals(REVLibError.kOk, enc.setPosition(0));
        assertEquals(0, enc.getPosition(), 0.02);
        positionSim.set(position / enc.getPositionConversionFactor() + positionSim.get());
        assertEquals(position, enc.getPosition(), 0.02);
    }

    private boolean simDeviceExists(String deviceName) {
        return Stream.of(SimDeviceSim.enumerateDevices(deviceName))
            .map(info -> info.name)
            .distinct()
            .filter(name -> name.equals(deviceName))
            .count() == 1;
    }

    private SafelyClosable createEncoder(int deviceId) {
        SimDevice device = SimDevice.create("testDevice", deviceId);
        return (SafelyClosable)Mocks.createMock(
            RelativeEncoder.class,
            new MockedEncoder(device, 4096, false, false),
            new REVLibErrorAnswer(),
            SafelyClosable.class);
    }

    private void withEncoders(EncoderTest func) {
        withEncoder(0, func);
        withEncoder(1, func);
        withEncoder(2, func);
    }

    private void withEncoder(int id, EncoderTest func) {
        try(SafelyClosable encoder = createEncoder(id)) {
            SimDeviceSim sim = new SimDeviceSim("testDevice", id);
            SimDouble posSim = sim.getDouble("position");
            assertNotNull(posSim);
            func.test((RelativeEncoder)encoder, sim, posSim);
        }
    }

    private interface EncoderTest {
        public void test(RelativeEncoder encoder, SimDeviceSim sim, SimDouble posSim);
    }
    
}
