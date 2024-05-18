package org.carlmontrobotics.lib199.sim;

import java.util.Arrays;

import org.carlmontrobotics.lib199.Mocks;

import com.playingwithfusion.TimeOfFlight.Status;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimEnum;
import edu.wpi.first.hal.SimInt;

public class MockedPlayingWithFusionTimeOfFlight implements AutoCloseable {

    private int port;
    private SimDevice device;
    private SimDouble range, rangeSigma, sampleTime, ambientLightLevel;
    private SimInt roiLeft, roiTop, roiRight, roiBottom;
    private SimEnum status;
    private SimEnum rangingMode;

    public MockedPlayingWithFusionTimeOfFlight(int portNumber) {
        port = portNumber;
        device = SimDevice.create("PlayingWithFusionTimeOfFlight", port);
        range = device.createDouble("range", Direction.kInput, 0);
        rangeSigma = device.createDouble("rangeSigma", Direction.kInput, 1);
        sampleTime = device.createDouble("sampleTime", Direction.kBidir, 24);

        // Note: default ambientLightLevel of 0.005*16*16 Mcps is typical for office lighting per the vl5311x datasheet:
        // https://www.playingwithfusion.com/include/getfile.php?fileid=7073
        ambientLightLevel = device.createDouble("ambientLightLevel", Direction.kInput, 0.005*16*16);

        String[] statusNames = Arrays.stream(Status.values()).map(Status::name).toArray(String[]::new);
        status = device.createEnum("status", Direction.kInput, statusNames, Status.Invalid.ordinal());

        String[] rangingModeNames = Arrays.stream(RangingMode.values()).map(RangingMode::name).toArray(String[]::new);
        rangingMode = device.createEnum("rangingMode", Direction.kInput, rangingModeNames, RangingMode.Short.ordinal());

        roiLeft = device.createInt("roiLeft", Direction.kOutput, 0);
        roiTop = device.createInt("roiTop", Direction.kOutput, 0);
        roiRight = device.createInt("roiRight", Direction.kOutput, 15);
        roiBottom = device.createInt("roiBottom", Direction.kOutput, 15);
    }

    public static TimeOfFlight createMock(int portNumber) {
        return Mocks.createMock(TimeOfFlight.class, new MockedPlayingWithFusionTimeOfFlight(portNumber));
    }

    public double getAmbientLightLevel() {
        return ambientLightLevel.get();
    }

    public double getRange() {
        return range.get();
    }

    public double getRangeSigma() {
        return rangeSigma.get();
    }

    public RangingMode getRangingMode() {
        return RangingMode.values()[rangingMode.get()];
    }

    public double getSampleTime() {
        return sampleTime.get();
    }

    public Status getStatus() {
        return Status.values()[status.get()];
    }

    public boolean isRangeValid() {
        return getStatus() == Status.Valid;
    }

    public double pidGet() {
        return getRange();
    }

    public void setRangeOfInterest(int topLeftX, int topLeftY, int bottomRightX, int bottomRightY) {
        roiLeft.set(topLeftX);
        roiTop.set(topLeftY);
        roiRight.set(bottomRightX);
        roiBottom.set(bottomRightY);
    }

    public void setRangingMode(RangingMode newMode, double newSampleTime) {
        rangingMode.set(newMode.ordinal());
        sampleTime.set(newSampleTime);
    }

    @Override
    public void close() {
        device.close();
    }
}
