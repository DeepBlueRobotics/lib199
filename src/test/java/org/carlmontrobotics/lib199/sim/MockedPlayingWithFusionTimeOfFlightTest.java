package org.carlmontrobotics.lib199.sim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.hamcrest.MatcherAssert.*;
import static org.hamcrest.CoreMatchers.*;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Stream;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.Status;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import org.carlmontrobotics.lib199.testUtils.SimDeviceTestRule;
import org.junit.ClassRule;
import org.junit.Rule;
import org.junit.Test;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimEnum;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class MockedPlayingWithFusionTimeOfFlightTest {

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
        String deviceName = String.format("PlayingWithFusionTimeOfFlight[%d]", id);
        assertFalse(simDeviceExists(deviceName));
        try(TimeOfFlight dev = createDevice(id)) {
            assertTrue(simDeviceExists(deviceName));
            SimDeviceSim sim = new SimDeviceSim("PlayingWithFusionTimeOfFlight", id);
            var valueNames = Stream.of(sim.enumerateValues()).map(info -> info.name).toList();
            assertThat(valueNames, hasItems("range", "rangeSigma", "sampleTime", "ambientLightLevel", "status", "rangingMode", "roiLeft", "roiTop", "roiRight", "roiBottom"));
        }
        assertFalse(simDeviceExists(deviceName));
    }

    @Test
    public void testRange() {
        withDevices((dev, sim) -> {
            SimDouble range = sim.getDouble("range");
            assertNotNull(range);
            dev.getRange(); // Default is not specified but must not throw.

            for (double val : new double[] {15.0, 150.0, 1000.0}) {
                range.set(val);
                assertThat(dev.getRange(), is(val));
                assertThat(dev.pidGet(), is(val));
            }
        });
    }

    @Test
    public void testRangeSigma() {
        withDevices((dev, sim) -> {
            SimDouble rangeSigma = sim.getDouble("rangeSigma");
            assertNotNull(rangeSigma);
            dev.getRangeSigma(); // Default is not specified but must not throw.

            for (double val : new double[] {15.0, 150.0, 1000.0}) {
                rangeSigma.set(val);
                assertThat(dev.getRangeSigma(), is(val));
            }
        });
    }

    @Test
    public void testStatus() {
        withDevices((dev, sim) -> {
            SimEnum status = sim.getEnum("status");
            assertNotNull(status);
            dev.getStatus(); // Default is not specified but must not throw.

            for (Status val : Status.values()) {
                status.set(val.ordinal());
                assertThat(dev.getStatus(), is(val));
                assertThat(dev.isRangeValid(), is(val == Status.Valid));
            }
        });
    }

    @Test
    public void testAmbientLightLevel() {
        withDevices((dev, sim) -> {
            SimDouble ambientLightLevel = sim.getDouble("ambientLightLevel");
            assertNotNull(ambientLightLevel);
            dev.getAmbientLightLevel(); // Default is not specified but must not throw.

            for (double val : new double[] {15.0, 150.0, 1000.0}) {
                ambientLightLevel.set(val);
                assertThat(dev.getAmbientLightLevel(), is(val));
            }
        });
    }

    @Test
    public void testRangingMode() {
        withDevices((dev, sim) -> {
            SimDouble sampleTime = sim.getDouble("sampleTime");
            assertNotNull(sampleTime);
            assertThat(dev.getSampleTime(), is(24.0));

            SimEnum rangingMode = sim.getEnum("rangingMode");
            assertNotNull(rangingMode);
            assertThat(dev.getRangingMode(), is(RangingMode.Short));

            dev.setRangingMode(RangingMode.Medium, 100.0);
            assertThat(dev.getRangingMode(), is(RangingMode.Medium));
            assertThat(dev.getSampleTime(), is(100.0));
        });
    }

    @Test
    public void testRangeOfInterest() {
        withDevices((dev, sim) -> {
            List<SimInt> roiList = Arrays.stream(new String[] {"Left", "Top", "Right", "Bottom"}).map(side -> sim.getInt("roi"+side)).toList();
            assertThat(roiList, everyItem(notNullValue(SimInt.class)));
            assertEquals(0, roiList.get(0).get());
            assertEquals(0, roiList.get(1).get());
            assertEquals(15, roiList.get(2).get());
            assertEquals(15, roiList.get(3).get());

            dev.setRangeOfInterest(1, 2, 5, 6);
            assertEquals(1, roiList.get(0).get());
            assertEquals(2, roiList.get(1).get());
            assertEquals(5, roiList.get(2).get());
            assertEquals(6, roiList.get(3).get());
        });
    }

    private boolean simDeviceExists(String deviceName) {
        return Stream.of(SimDeviceSim.enumerateDevices(deviceName))
            .map(info -> info.name)
            .distinct()
            .filter(name -> name.equals(deviceName))
            .count() == 1;
    }

    private TimeOfFlight createDevice(int deviceId) {
        return MockedPlayingWithFusionTimeOfFlight.createMock(deviceId);
    }

    private void withDevices(EncoderTest func) {
        withDevice(0, func);
        withDevice(1, func);
        withDevice(2, func);
    }

    private void withDevice(int id, EncoderTest func) {
        try(TimeOfFlight dev = createDevice(id)) {
            SimDeviceSim sim = new SimDeviceSim("PlayingWithFusionTimeOfFlight", id);
            func.test((TimeOfFlight)dev, sim);
        }
    }

    private interface EncoderTest {
        public void test(TimeOfFlight encoder, SimDeviceSim sim);
    }
    
}
