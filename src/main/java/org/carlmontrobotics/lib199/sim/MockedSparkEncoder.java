package org.carlmontrobotics.lib199.sim;

import java.util.HashMap;

import org.carlmontrobotics.lib199.Lib199Subsystem;

import com.revrobotics.REVLibError;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;

public class MockedSparkEncoder implements AutoCloseable, Runnable {

    private static final HashMap<Integer, MockedSparkEncoder> sims = new HashMap<>();

    private SimDevice device;
    private SimDouble count;
    private SimDouble gearing;
    // Default value for a CANEncoder
    private final int countsPerRevolution = 4096;
    private double velocity;
    private double positionConversionFactor = 1;
    private double velocityConversionFactor = 1;
    private double lastCount = 0;
    private double lastTime = 0;

    public MockedSparkEncoder(int id) {
        device = SimDevice.create("RelativeEncoder", id);
        count = device.createDouble("count", Direction.kInput, 0);
        gearing = device.createDouble("gearing", Direction.kOutput, 1);
        sims.put(id, this);
        Lib199Subsystem.registerPeriodic(this);
    }

    public double getPosition() {
        return positionConversionFactor * count.get() / countsPerRevolution;
    }

    public REVLibError setPositionConversionFactor(double positionConversionFactor) {
        this.positionConversionFactor = positionConversionFactor;
        return REVLibError.kOk;
    }

    public REVLibError setPosition(double position) {
        double revolutions = position / getPositionConversionFactor();
        count.set((int) Math.floor(revolutions * countsPerRevolution));
        return REVLibError.kOk;
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
        double curCount = count.get();
        double dCount = curCount - lastCount;
        lastTime = t;
        lastCount = curCount;
        double newVelocity = velocityConversionFactor * ( dCount / dt ) / countsPerRevolution;
        velocity = Double.isNaN(newVelocity) ? 0 : newVelocity;
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