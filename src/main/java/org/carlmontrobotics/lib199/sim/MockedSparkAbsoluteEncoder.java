package org.carlmontrobotics.lib199.sim;

import java.util.HashMap;

import org.carlmontrobotics.lib199.Lib199Subsystem;

import com.revrobotics.REVLibError;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;

public class MockedSparkAbsoluteEncoder implements AutoCloseable, Runnable {

    private static final HashMap<Integer, MockedSparkAbsoluteEncoder> sims = new HashMap<>();

    private SimDevice device;
    private SimDouble rotations;
    private double velocity;
    private double positionConversionFactor = 1;
    private double velocityConversionFactor = 1;
    private double lastRotations = 0;
    private double lastTime = 0;
    private double zeroOffset = 0.0;

    public MockedSparkAbsoluteEncoder(int id) {
        device = SimDevice.create("AbsoluteEncoder", id);
        rotations = device.createDouble("rotations", Direction.kInput, 0);
        sims.put(id, this);
        Lib199Subsystem.registerAsyncPeriodic(this);
    }

    public double getPosition() {
        return positionConversionFactor * rotations.get() - zeroOffset;
    }

    public REVLibError setPositionConversionFactor(double positionConversionFactor) {
        this.positionConversionFactor = positionConversionFactor;
        return REVLibError.kOk;
    }

    public REVLibError setZeroOffset(double zeroOffset) {
        this.zeroOffset = zeroOffset;
        return REVLibError.kOk;
    }

    public double getZeroOffset() {
        return zeroOffset;
    }

    public double getPositionConversionFactor() {
        return positionConversionFactor;
    }

    public double getVelocity() {
        return velocity;
    }

    public REVLibError setVelocityConversionFactor(double velocityConversionFactor) {
        this.velocityConversionFactor = velocityConversionFactor;
        return REVLibError.kOk;
    }

    public double getVelocityConversionFactor() {
        return velocityConversionFactor;
    }

    @Override
    public void run() {
        double t = System.currentTimeMillis() / 1000D;
        double dt = t - lastTime;
        double curRotations = rotations.get();
        double dCount = curRotations - lastRotations;
        lastTime = t;
        lastRotations = curRotations;
        double newVelocity = velocityConversionFactor * ( dCount / dt ) * 60;
        velocity = Double.isNaN(newVelocity) ? 0 : newVelocity;
    }

    @Override
    public void close() {
        device.close();
    }
}