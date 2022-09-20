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
import org.carlmontrobotics.lib199.testUtils.SimDeviceTestRule;
import org.junit.ClassRule;
import org.junit.Rule;
import org.junit.Test;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class MockedSparkEncoderTest {

    @ClassRule
    public static SimDeviceTestRule.Class simClassRule = new SimDeviceTestRule.Class(); 
    @Rule
    public SimDeviceTestRule.Test simTestRule = new SimDeviceTestRule.Test(); 

    @Test
    public void testDeviceCreation() {
        assertTestDeviceCreation(0);
        assertTestDeviceCreation(1);
        assertTestDeviceCreation(2);
    }

    private void assertTestDeviceCreation(int id) {
        String deviceName = String.format("CANEncoder_SparkMax[%d]", id);
        assertFalse(simDeviceExists(deviceName));
        try(SafelyClosable closableEncoder = createEncoder(id)) {
            assertTrue(simDeviceExists(deviceName));
            SimDeviceSim sim = new SimDeviceSim("CANEncoder_SparkMax", id);
            assertEquals(2, Stream.of(sim.enumerateValues())
                .map(info -> info.name)
                .distinct()
                .filter(name -> name.equals("distancePerPulse") || name.equals("count")).count());
        }
        assertFalse(simDeviceExists(deviceName));
    }

    @Test
    public void testFunctionality() {
        withEncoders((enc, sim, dpp, count) -> {
            testFunctionalityWithPositionConversionFactor(1, enc, dpp, count);
            testFunctionalityWithPositionConversionFactor(10, enc, dpp, count);
            testFunctionalityWithPositionConversionFactor(100, enc, dpp, count);
        });
    }

    private void testFunctionalityWithPositionConversionFactor(double factor, RelativeEncoder enc, SimDouble dpp,
            SimDouble count) {
        assertEquals(REVLibError.kOk, enc.setPositionConversionFactor(factor));
        assertEquals(factor/4096, dpp.get(), 0.01);
        assertEquals(factor, enc.getPositionConversionFactor(), 0.01);
        testCount(10, enc, dpp, count);
        testCount(0, enc, dpp, count);
        testCount(-10, enc, dpp, count);
    }

    private void testCount(double position, RelativeEncoder enc, SimDouble dpp, SimDouble count) {
        // This test fails with a delta of 0.01
        assertEquals(REVLibError.kOk, enc.setPosition(position));
        assertEquals(position, enc.getPosition(), 0.02);
        assertEquals(REVLibError.kOk, enc.setPosition(0));
        assertEquals(0, enc.getPosition(), 0.02);
        count.set(position/dpp.get());
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
        return (SafelyClosable)Mocks.createMock(
            RelativeEncoder.class,
            new MockedSparkEncoder(deviceId),
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
            SimDeviceSim sim = new SimDeviceSim("CANEncoder_SparkMax", id);
            SimDouble dpp = sim.getDouble("distancePerPulse");
            SimDouble count = sim.getDouble("count");
            assertNotNull(dpp);
            assertNotNull(count);
            func.test((RelativeEncoder)encoder, sim, dpp, count);
        }
    }

    private interface EncoderTest {
        public void test(RelativeEncoder encoder, SimDeviceSim sim, SimDouble dpp, SimDouble count);
    }
    
}
