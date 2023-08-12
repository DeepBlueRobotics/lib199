package org.carlmontrobotics.lib199.sim;

import org.carlmontrobotics.lib199.Lib199Subsystem;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class MockedMotorBase implements Runnable {

    public final SimDevice device;
    public final SimDouble speed;
    public final SimDouble neutralDeadband;
    public final SimBoolean brakeModeEnabled;
    private SlewRateLimiter rampRateLimiter = null;
    private double requestedSpeedPercent = 0.0;

    public MockedMotorBase(String type, int port) {
        device = SimDevice.create(type, port);
        speed = device.createDouble("Speed", Direction.kOutput, 0.0);
        neutralDeadband = device.createDouble("Neutral Deadband", Direction.kOutput, 0.04);
        brakeModeEnabled = device.createBoolean("Brake Mode", Direction.kOutput, true);

        Lib199Subsystem.registerAsyncSimulationPeriodic(this);
    }

    public void set(double percent) {
        requestedSpeedPercent = percent;
    }

    public void setNeutralDeadband(double deadbandPercent) {
        this.neutralDeadband.set(Math.abs(deadbandPercent));
    }

    public void setBrakeMode(boolean brakeMode) {
        this.brakeModeEnabled.set(brakeMode);
    }

    public void setRampRate(double rampRatePercentPerSec) {
        rampRatePercentPerSec = Math.abs(rampRatePercentPerSec);
        rampRateLimiter = new SlewRateLimiter(rampRatePercentPerSec, -rampRatePercentPerSec, speed.get());
    }

    @Override
    public void run() {
        speed.set(rampRateLimiter.calculate(requestedSpeedPercent));
    }

}
