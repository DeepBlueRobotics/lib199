package org.carlmontrobotics.lib199.sim;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * Represents a base encoder class which can connect to a DeepBlueSim SimDeviceMotorMediator.
 *
 * This class implements all {@link MotorController} methods except for {@link MotorController#set(double)}.
 * Instead, subclasses implement the {@link #getRequestedSpeed()} method which is called periodically to set the
 * speed of the simulated motor.
 *
 * Currently this is only used for spark max simulation, pending #62.
 */
public abstract class MockedMotorBase implements AutoCloseable, MotorController, Runnable {

    public static final double defaultNominalVoltage = 12.0;

    public final SimDevice device;
    public final int port;
    public final SimDouble speed;
    public final SimDouble neutralDeadband;
    public final SimBoolean brakeModeEnabled;
    public final SimDouble currentDraw;
    public final SimDouble busVoltage;
    protected SlewRateLimiter rampRateLimiter = null;
    protected boolean isInverted = false;
    protected boolean disabled = false;
    protected double voltageCompensationNominalVoltage = defaultNominalVoltage;
    protected double closedLoopRampRate = 0.0;
    protected double openLoopRampRate = 0.0;
    protected boolean runningClosedLoopControl = false;
    private double requestedSpeedPercent = 0.0;

    /**
     * Initializes a new {@link SimDevice} with the given parameters and creates the necessary sim values.
     *
     * @param type the device type name to pass to {@link SimDevice#create}
     * @param port the device port to pass to {@link SimDevice#create}
     */
    public MockedMotorBase(String type, int port) {
        device = SimDevice.create("CANMotor:" + type, port);
        this.port = port;
        speed = device.createDouble("percentOutput", Direction.kOutput, 0.0);
        neutralDeadband = device.createDouble("neutralDeadband", Direction.kOutput, 0.04);
        brakeModeEnabled = device.createBoolean("brakeMode", Direction.kOutput, true);
        currentDraw = device.createDouble("motorCurrent", Direction.kInput, 0.0);
        busVoltage = device.createDouble("busVoltage", Direction.kInput, defaultNominalVoltage);
    }

    /**
     * Sets the speed range in which this controller will be set to break mode.
     * This value should be in the range [0, 1].
     *
     * @param deadbandPercent the range in which this controller will be considered stopped
     */
    public void setNeutralDeadband(double deadbandPercent) {
        this.neutralDeadband.set(Math.abs(deadbandPercent));
    }

    /**
     * Sets whether this controller should be in brake mode or coast mode when idle.
     *
     * @param brakeMode whether this controller should be in brake mode
     */
    public void setBrakeModeEnabled(boolean brakeMode) {
        this.brakeModeEnabled.set(brakeMode);
    }

    // The ramp rate method names look weird, but this is just to prevent clashing with the vendor methods

    /**
     * Sets the ramp rate of this controller.
     * @param secondsFromNeutralToFull the number of seconds it should take to go from 0 to full speed
     */
    public void setRampRate(double secondsFromNeutralToFull) {
        if(secondsFromNeutralToFull <= 0) {
            rampRateLimiter = null;
            return;
        }
        double rateLimit = voltageCompensationNominalVoltage / secondsFromNeutralToFull;
        rampRateLimiter = new SlewRateLimiter(rateLimit, -rateLimit, speed.get());
    }

    /**
     * Sets the ramp rate of this controller when in a closed loop control mode.
     * @param secondsFromNeutralToFull the number of seconds it should take to go from 0 to full speed
     */
    public void setRampRateClosedLoop(double secondsFromNeutralToFull) {
        closedLoopRampRate = secondsFromNeutralToFull;
        if(runningClosedLoopControl) setRampRate(secondsFromNeutralToFull);
    }

    /**
     * Sets the ramp rate of this controller when in an open loop control mode.
     * @param secondsFromNeutralToFull the number of seconds it should take to go from 0 to full speed
     */
    public void setRampRateOpenLoop(double secondsFromNeutralToFull) {
        openLoopRampRate = secondsFromNeutralToFull;
        if(!runningClosedLoopControl) setRampRate(secondsFromNeutralToFull);
    }

    /**
     * Sets whether this controller is running in a closed or open loop loop control mode.
     * @param enabled whether this controller is running in a closed loop control mode
     */
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
        voltageCompensationNominalVoltage = busVoltage.get();
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
        percent *= voltageCompensationNominalVoltage / busVoltage.get();
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
        disabled = true;
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
