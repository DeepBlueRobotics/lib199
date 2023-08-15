package org.carlmontrobotics.lib199.sim;

import org.carlmontrobotics.lib199.Lib199Subsystem;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public abstract class MockedMotorBase implements AutoCloseable, MotorController, Runnable {

    public static final double defaultNominalVoltage = 12.0;

    public final SimDevice device;
    public final int port;
    public final SimDouble speed;
    public final SimDouble neutralDeadband;
    public final SimBoolean brakeModeEnabled;
    public final SimDouble currentDraw;
    public final boolean allowMotorDisable;
    protected SlewRateLimiter rampRateLimiter = null;
    protected boolean isInverted = false;
    protected boolean disabled = false;
    protected double voltageCompensationNominalVoltage = defaultNominalVoltage;
    protected double closedLoopRampRate = 0.0;
    protected double openLoopRampRate = 0.0;
    protected boolean runningClosedLoopControl = false;
    private double requestedSpeedPercent = 0.0;

    public MockedMotorBase(String type, int port, boolean allowMotorDisable) {
        device = SimDevice.create(type, port);
        this.port = port;
        speed = device.createDouble("Speed", Direction.kOutput, 0.0);
        neutralDeadband = device.createDouble("Neutral Deadband", Direction.kOutput, 0.04);
        brakeModeEnabled = device.createBoolean("Brake Mode", Direction.kOutput, true);
        currentDraw = device.createDouble("Current Draw", Direction.kInput, 0.0);
        this.allowMotorDisable = allowMotorDisable;

        Lib199Subsystem.registerAsyncSimulationPeriodic(this);
    }

    public void setNeutralDeadband(double deadbandPercent) {
        this.neutralDeadband.set(Math.abs(deadbandPercent));
    }

    public void setBrakeModeEnabled(boolean brakeMode) {
        this.brakeModeEnabled.set(brakeMode);
    }

    // The ramp rate method names look weird, but this is just to prevent clashing with the vendor methods

    public void setRampRate(double secondsFromNeutralToFull) {
        if(secondsFromNeutralToFull <= 0) {
            rampRateLimiter = null;
            return;
        }
        double rateLimit = voltageCompensationNominalVoltage / secondsFromNeutralToFull;
        rampRateLimiter = new SlewRateLimiter(rateLimit, -rateLimit, speed.get());
    }

    public void setRampRateClosedLoop(double secondsFromNeutralToFull) {
        closedLoopRampRate = secondsFromNeutralToFull;
        if(runningClosedLoopControl) setRampRate(secondsFromNeutralToFull);
    }

    public void setRampRateOpenLoop(double secondsFromNeutralToFull) {
        openLoopRampRate = secondsFromNeutralToFull;
        if(!runningClosedLoopControl) setRampRate(secondsFromNeutralToFull);
    }

    public void setClosedLoopControl(boolean enabled) {
        runningClosedLoopControl = enabled;
        if(enabled) {
            setRampRate(closedLoopRampRate);
        } else {
            setRampRate(openLoopRampRate);
        }
    }

    public boolean getClosedLoopControl() {
        return runningClosedLoopControl;
    }

    public double getRampRate() {
        return runningClosedLoopControl ? closedLoopRampRate : openLoopRampRate;
    }

    public double getRampRateClosedLoop() {
        return closedLoopRampRate;
    }

    public double getRampRateOpenLoop() {
        return openLoopRampRate;
    }

    public void doEnableVoltageCompensation(double nominalVoltage) {
        voltageCompensationNominalVoltage = nominalVoltage;
        setRampRate(getRampRate()); // Update the ramp rate to account for the new nominal voltage
	}

	public void doDisableVoltageCompensation() {
        voltageCompensationNominalVoltage = defaultNominalVoltage;
        setRampRate(getRampRate()); // Update the ramp rate to account for the new nominal voltage
    }

    public double getVoltageCompensationNominalVoltage() {
        return voltageCompensationNominalVoltage;
    }

    public double getCurrentDraw() {
        return currentDraw.get();
    }

    public void updateRequestedSpeed() {
        double percent = getRequestedSpeed();
        percent *= voltageCompensationNominalVoltage / defaultNominalVoltage;
        percent *= isInverted ? -1.0 : 1.0;
        requestedSpeedPercent = percent;
    }

    @Override
    public void run() {
        if (disabled) {
            requestedSpeedPercent = 0;
            speed.set(0);
        } else {
            updateRequestedSpeed();
            if (rampRateLimiter == null) {
                speed.set(requestedSpeedPercent);
            } else {
                speed.set(rampRateLimiter.calculate(requestedSpeedPercent));
            }
        }
    }

    public abstract double getRequestedSpeed();

    @Override
    public double get() {
        return (isInverted ? -1.0 : 1.0) * requestedSpeedPercent;
    }

    @Override
    public void setInverted(boolean isInverted) {
        this.isInverted = isInverted;
    }

    @Override
    public boolean getInverted() {
        return isInverted;
    }

    @Override
    public void disable() {
        if (allowMotorDisable) {
            disabled = true;
        }
        set(0);
    }

    @Override
    public void stopMotor() {
        set(0);
    }

    @Override
    public void setVoltage(double outputVolts) {
        set(outputVolts / voltageCompensationNominalVoltage);
    }

    @Override
    public void close() {
        device.close();
    }

}
