package org.carlmontrobotics.lib199.sim;

import java.util.HashMap;

import com.revrobotics.REVLibError;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;

public class MockedSparkEncoder implements AutoCloseable {

    private static final HashMap<Integer, MockedSparkEncoder> sims = new HashMap<>();

    private SimDevice device;
    private SimDouble dpp;
    private SimDouble count;
    private SimDouble gearing;
    // Default value for a CANEncoder
    private final int countsPerRevolution = 4096;

    public MockedSparkEncoder(int id) {
        device = SimDevice.create("RelativeEncoder", id);
        dpp = device.createDouble("distancePerPulse", Direction.kOutput, 1);
        count = device.createDouble("count", Direction.kInput, 0);
        gearing = device.createDouble("gearing", Direction.kOutput, 1);
        sims.put(id, this);
    }

    public double getPosition() {
        return dpp.get() * count.get();
    }

    public REVLibError setPositionConversionFactor(double positionConversionFactor) {
        // Assume positionConversionFactor = units/rev
        // distancePerPulse (actually distance per count) = units/rev * rev/count
        dpp.set(positionConversionFactor / countsPerRevolution);
        return REVLibError.kOk;
    }

    public REVLibError setPosition(double position) {
        double revolutions = position / getPositionConversionFactor();
        count.set((int) Math.floor(revolutions * countsPerRevolution));
        return REVLibError.kOk;
    }

    public double getPositionConversionFactor() {
        return dpp.get() * countsPerRevolution;
    }

    @Override
    public void close() {
        device.close();
    }

    public void setGearing(double gearing) {
        this.gearing.set(gearing);
    }

    public static void setGearing(int port, double gearing) {
        if(sims.containsKey(port)) sims.get(port).setGearing(gearing);
    }

}