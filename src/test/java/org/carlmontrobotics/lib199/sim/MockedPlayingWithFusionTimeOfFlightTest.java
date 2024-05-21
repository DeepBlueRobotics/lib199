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

import org.carlmontrobotics.lib199.testUtils.TestRules;
import org.junit.ClassRule;
import org.junit.Rule;
import org.junit.Test;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimEnum;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class MockedPlayingWithFusionTimeOfFlightTest {

    @ClassRule
    public static TestRules.InitializeHAL classRule = new TestRules.InitializeHAL();
    @Rule
    public TestRules.ResetSimDeviceSimData testRule = new TestRules.ResetSimDeviceSimData();

    @Test
    public void testDeviceCreation() {
        assertTestDeviceCreation(0);
        assertTestDeviceCreation(1);
        assertTestDeviceCreation(2);
    }

    private void assertTestDeviceCreation(int id) {
        String deviceName = String.format("CANAIn:PlayingWithFusionTimeOfFlight[%d]-rangeVoltsIsMM", id);
        assertFalse(simDeviceExists(deviceName));
        try(TimeOfFlight dev = createDevice(id)) {
            assertTrue(simDeviceExists(deviceName));
            SimDeviceSim sim = new SimDeviceSim(deviceName);
            var valueNames = Stream.of(sim.enumerateValues()).map(info -> info.name).toList();
            assertThat(valueNames, hasItems("voltage", "rangeSigma", "sampleTime", "status", "rangingMode", "roiLeft", "roiTop", "roiRight", "roiBottom"));
        }
        assertFalse(simDeviceExists(deviceName));
    }

    @Test
    public void testRange() {
        withDevices((dev, rangeDeviceSim, ambientLightLevelDeviceSim) -> {
            SimDouble range = rangeDeviceSim.getDouble("voltage");
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
        withDevices((dev, rangeDeviceSim, ambientLightLevelDeviceSim) -> {
            SimDouble rangeSigma = rangeDeviceSim.getDouble("rangeSigma");
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
        withDevices((dev, rangeDeviceSim, ambientLightLevelDeviceSim) -> {
            SimEnum status = rangeDeviceSim.getEnum("status");
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
        withDevices((dev, rangeDeviceSim, ambientLightLevelDeviceSim) -> {
            SimDouble ambientLightLevel = ambientLightLevelDeviceSim.getDouble("voltage");
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
        withDevices((dev, rangeDeviceSim, ambientLightLevelDeviceSim) -> {
            SimDouble sampleTime = rangeDeviceSim.getDouble("sampleTime");
            assertNotNull(sampleTime);
            assertThat(dev.getSampleTime(), is(24.0));

            SimEnum rangingMode = rangeDeviceSim.getEnum("rangingMode");
            assertNotNull(rangingMode);
            assertThat(dev.getRangingMode(), is(RangingMode.Short));

            dev.setRangingMode(RangingMode.Medium, 100.0);
            assertThat(dev.getRangingMode(), is(RangingMode.Medium));
            assertThat(dev.getSampleTime(), is(100.0));
        });
    }

    @Test
    public void testRangeOfInterest() {
        withDevices((dev, rangeDeviceSim, ambientLightLevelDeviceSim) -> {
            List<SimInt> roiList = Arrays.stream(new String[] {"Left", "Top", "Right", "Bottom"}).map(side -> rangeDeviceSim.getInt("roi"+side)).toList();
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
            SimDeviceSim rangeDeviceSim = new SimDeviceSim(String.format("CANAIn:PlayingWithFusionTimeOfFlight[%d]-rangeVoltsIsMM", id));
            SimDeviceSim ambientLightLevelDeviceSim = new SimDeviceSim(String.format("CANAIn:PlayingWithFusionTimeOfFlight[%d]-ambientLightLevelVoltsIsMcps", id));
            func.test((TimeOfFlight)dev, rangeDeviceSim, ambientLightLevelDeviceSim);
        }
    }

    private interface EncoderTest {
        public void test(TimeOfFlight encoder, SimDeviceSim rangeDeviceSim, SimDeviceSim ambientLightLevelDeviceSim);
    }

}
