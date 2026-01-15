package org.carlmontrobotics.lib199.sim;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.MAXMotionConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

// NOT THREAD SAFE
public class MockedSparkClosedLoopController {

    public final Map<Integer, Slot> slots = new ConcurrentHashMap<>();
    public final MockedMotorBase motor;
    public Slot activeSlot;
    public ClosedLoopSlot activeClosedLoopSlot;
    public SparkMax.ControlType controlType = SparkMax.ControlType.kDutyCycle;
    public MotorController leader = null;
    public boolean invertLeader = false;
    public double setpoint = 0.0;
    public double arbFF = 0.0;
    public FeedbackDevice feedbackDevice;
    public boolean positionPIDWrappingEnabled = false;
    public double positionPIDWrappingMinInput = 0.0;
    public double positionPIDWrappingMaxInput = 0.0;

    public MockedSparkClosedLoopController(MockedMotorBase motor) {
        this.motor = motor;
        slots.put(0, activeSlot = new Slot(positionPIDWrappingMinInput, positionPIDWrappingMaxInput, positionPIDWrappingEnabled));
    }

    //Actually Real Methods

    /** Get the selected control type used when setReference(double, SparkBase.ControlType) was last called.*/
    public SparkBase.ControlType getControlType() {
        return controlType;
    }

    /** Get the I accumulator of the closed loop controller. */
    public double getIAccum() {
        System.err.println("(MockedSparkMaxPIDController): getIAccum() is not currently implemented");
        return 0;
    }

    /**Get the MAXMotion internal setpoint position. */
    public double getMAXMotionSetpointPosition() {
        return setpoint;
    }

    /**Get the MAXMotion internal setpoint velocity. */
    public double getMAXMotionSetpointVelocity() {
        return setpoint;
    }

    /** Get the selected closed loop PID slot. */
    public ClosedLoopSlot getSelectedSlot() {
        return activeClosedLoopSlot;
    }

    /**Get the internal setpoint of the closed loop controller. */
    public double getSetpoint() {
        return setpoint;
    }

    /** Determine if the setpoint has been reached.*/
    public boolean isAtSetpoint() {return false;}

    /** Set the I accumulator of the closed loop controller. */
    public REVLibError setIAccum(double iAccum) {
        System.err.println("(MockedSparkMaxPIDController): setIAccum() is not currently implemented");
        return REVLibError.kNotImplemented;
    }
    /** Deprecated, for removal: This API element is subject to removal in a future version.
     * Use {@link #setSetpoint(double, SparkBase.ControlType)} instead
    */
    @Deprecated
    public REVLibError setReference(double value, SparkMax.ControlType ctrl) {
        return setReference(value, ctrl, ClosedLoopSlot.kSlot0);
    }
    /** Deprecated, for removal: This API element is subject to removal in a future version.
     * Use {@link #setSetpoint(double, SparkBase.ControlType, ClosedLoopSlot)} instead
    */
    @Deprecated
    public REVLibError setReference(double value, SparkMax.ControlType ctrl, ClosedLoopSlot pidSlot) {
        return setReference(value, ctrl, pidSlot, 0);
    }
    /** Deprecated, for removal: This API element is subject to removal in a future version.
     * Use {@link #setSetpoint(double, SparkBase.ControlType, ClosedLoopSlot, double)} instead*/
    @Deprecated
    public REVLibError setReference(double value, SparkMax.ControlType ctrl, ClosedLoopSlot pidSlot, double arbFeedforward) {
        return setReference(value, ctrl, pidSlot, arbFeedforward, SparkClosedLoopController.ArbFFUnits.kVoltage);
    }
    /** Deprecated, for removal: This API element is subject to removal in a future version.
     * Use {@link #setSetpoint(double, SparkBase.ControlType, ClosedLoopSlot, double, ArbFFUnits)} instead
    */
    @Deprecated
    public REVLibError setReference(double value, SparkMax.ControlType ctrl, ClosedLoopSlot pidSlot, double arbFeedforward, SparkClosedLoopController.ArbFFUnits arbFFUnits) {
        if(ctrl == SparkMax.ControlType.kSmartVelocity) {
            System.err.println("(MockedSparkMaxPIDController): setReference() with ControlType.kSmartVelocity is not currently implemented");
            return REVLibError.kNotImplemented;
        }

        setpoint = value;
        controlType = ctrl;
        leader = null;

        switch(ctrl) {
            case kDutyCycle:
            case kVoltage:
                motor.setClosedLoopControl(false);
                break;
            case kPosition:
            case kVelocity:
            case kSmartMotion:
            case kMAXMotionPositionControl:
            case kMAXMotionVelocityControl:
            case kCurrent:
                motor.setClosedLoopControl(true);
                break;
            case kSmartVelocity:
                break; // This should never happen
        }

        activeSlot = getSlot(pidSlot.value);
        activeClosedLoopSlot = pidSlot;

        switch(arbFFUnits) {
            case kVoltage:
                break;
            case kPercentOut:
                arbFeedforward *= 12.0;
                break;
            default:
                throw new IllegalArgumentException("Unsupported ArbFFUnits: " + arbFFUnits);
        }

        this.arbFF = arbFeedforward;

        return REVLibError.kOk;
    }
    REVLibError setSetpoint(double setpoint, SparkBase.ControlType ctrl) {return null;}//Set the controller setpoint based on the selected control mode.
    REVLibError setSetpoint(double setpoint, SparkBase.ControlType ctrl, ClosedLoopSlot slot) {return null;}//Set the controller setpoint based on the selected control mode.
    REVLibError setSetpoint(double setpoint, SparkBase.ControlType ctrl, ClosedLoopSlot slot, double arbFeedforward) {return null;}//Set the controller setpoint based on the selected control mode.
    REVLibError setSetpoint(double setpoint, SparkBase.ControlType ctrl, ClosedLoopSlot slot, double arbFeedforward, SparkClosedLoopController.ArbFFUnits arbFFUnits) {return null;}//Set the controller setpoint based on the selected control mode.

    public interface ExtraMethods {
        double calculate(double currentDraw);
        void setDutyCycle(double speed);
        void follow(MotorController leader, boolean invert);
        void stopFollowing();
        boolean isFollower();
        double getD();
        double getD(int slotID);
        double getFF();
        double getFF(int slotID);
        double getI();
        double getI(int slotID);
        double getIMaxAccum(int slotID);
        double getIZone();
        double getIZone(int slotID);
        double getOutputMax();
        double getOutputMax(int slotID);
        double getOutputMin();
        double getOutputMin(int slotID);
        boolean getPositionPIDWrappingEnabled();
        double getPositionPIDWrappingMaxInput();
        double getPositionPIDWrappingMinInput();
        REVLibError setPositionPIDWrappingEnabled(boolean enable);
        REVLibError setPositionPIDWrappingMaxInput(double max);
        REVLibError setPositionPIDWrappingMinInput(double min);
        double getP();
        double getP(int slotID);
        MAXMotionConfig.MAXMotionPositionMode getSmartMotionAccelStrategy(int slotID);
        double getSmartMotionAllowedClosedLoopError(int slotID);
        double getSmartMotionMaxAccel(int slotID);
        double getSmartMotionMaxVelocity(int slotID);
        double getSmartMotionMinOutputVelocity(int slotID);
        REVLibError setD(double gain);
        REVLibError setD(double gain, int slotID);
        REVLibError setFeedbackDevice(Object sensor);
        REVLibError setFF(double gain);
        REVLibError setFF(double gain, int slotID);
        REVLibError setI(double gain);
        REVLibError setI(double gain, int slotID);
        REVLibError setIMaxAccum(double iMaxAccum, int slotID);
        REVLibError setIZone(double IZone);
        REVLibError setIZone(double IZone, int slotID);
        REVLibError setOutputRange(double min, double max);
        REVLibError setOutputRange(double min, double max, int slotID);
        REVLibError setP(double gain);
        REVLibError setP(double gain, int slotID);
        REVLibError setSmartMotionAccelStrategy(MAXMotionConfig.MAXMotionPositionMode accelStrategy, int slotID);
        REVLibError setSmartMotionAllowedClosedLoopError(double allowedErr, int slotId);
        REVLibError setSmartMotionMaxAccel(double maxAccel, int slotID);
        REVLibError setSmartMotionMaxVelocity(double maxVel, int slotID);
        REVLibError setSmartMotionMinOutputVelocity(double minVel, int slotID);
        Slot getSlot(int slotID);
    }


    // Methods to interface with MockSparkMax
    public double calculate(double currentDraw) {
        if(leader != null) {
            // For spark maxes, the follower receives the post-leader-inversion speed and does not depend on the inversion state from setInverted
            // For following inversion semantics see the "Motor Inversion Testing Results" section of the "Programming Resources/Documentation" document in the the team drive
            return (invertLeader ? -1 : 1) * leader.get();
        }

        double output = 0;
        switch(controlType) {
            case kDutyCycle:
                return setpoint;
            case kVoltage:
                return setpoint / 12.0;
            case kPosition:
                output = activeSlot.pidController.calculate(feedbackDevice.getPosition(), setpoint);
                break;
            case kVelocity:
                output = activeSlot.pidController.calculate(feedbackDevice.getVelocity(), setpoint);
                break;
            case kMAXMotionPositionControl:
                output = activeSlot.profiledPIDController.calculate(feedbackDevice.getPosition(), setpoint);
                if(Math.abs(activeSlot.profiledPIDController.getGoal().velocity) < activeSlot.smartMotionMinVelocity) {
                    output = 0;//FIXME max motion doesn't have min velocity!!!
                }
                break;
            case kCurrent:
                output = activeSlot.pidController.calculate(currentDraw, setpoint);
                break;
            case kMAXMotionVelocityControl:
            default:
                throw new IllegalArgumentException("Unsupported ControlType: " + controlType);
        }
        output += activeSlot.ff * setpoint + arbFF;
        output = MathUtil.clamp(output, activeSlot.outputMin, activeSlot.outputMax);
        return output;
    }

    public void setDutyCycle(double speed) {
        setpoint = speed;
        controlType = SparkMax.ControlType.kDutyCycle;
        stopFollowing();
        motor.setClosedLoopControl(false);
    }

    public void follow(MotorController leader, boolean invert) {
        this.leader = leader;
        invertLeader = invert;
        motor.setClosedLoopControl(false);
    }

    public void stopFollowing() {
        this.leader = null;
    }

    public boolean isFollower() {
        return leader != null;
    }

    // Overrides
    public double getD() {
        return getD(0);
    }

    public double getD(int slotID) {
        Slot slot = getSlot(slotID);
        return slot.pidController.getD();
    }

    public double getFF() {
        return getFF(0);
    }

    public double getFF(int slotID) {
        return getSlot(slotID).ff;
    }

    public double getI() {
        return getI(0);
    }

    public double getI(int slotID) {
        Slot slot = getSlot(slotID);
        return slot.pidController.getI();
    }

    public double getIMaxAccum(int slotID) {
        return getSlot(slotID).iMaxAccum;
    }

    public double getIZone() {
        return getIZone(0);
    }

    public double getIZone(int slotID) {
        return 0;
    }

    public double getOutputMax() {
        return getOutputMax(0);
    }

    public double getOutputMax(int slotID) {
        return getSlot(slotID).outputMax;
    }

    public double getOutputMin() {
        return getOutputMin(0);
    }

    public double getOutputMin(int slotID) {
        return getSlot(slotID).outputMin;
    }

    public boolean getPositionPIDWrappingEnabled() {
        return positionPIDWrappingEnabled;

    }

    public double getPositionPIDWrappingMaxInput() {
        return positionPIDWrappingMaxInput;
    }

    public double getPositionPIDWrappingMinInput() {
        return positionPIDWrappingMinInput;
    }

    private void updatePIDWrapping() {
        slots.values().forEach(slot -> {
            if(positionPIDWrappingEnabled) {
                slot.pidController.enableContinuousInput(positionPIDWrappingMinInput, positionPIDWrappingMaxInput);
                slot.profiledPIDController.enableContinuousInput(positionPIDWrappingMinInput, positionPIDWrappingMaxInput);
            } else {
                slot.pidController.disableContinuousInput();
                slot.profiledPIDController.disableContinuousInput();
            }
        });
    }

    public REVLibError setPositionPIDWrappingEnabled(boolean enable) {
        if(enable == positionPIDWrappingEnabled) return REVLibError.kOk;
        positionPIDWrappingEnabled = enable;
        updatePIDWrapping();
        return REVLibError.kOk;
    }

    public REVLibError setPositionPIDWrappingMaxInput(double max) {
        if(max == positionPIDWrappingMaxInput) return REVLibError.kOk;
        positionPIDWrappingMaxInput = max;
        updatePIDWrapping();
        return REVLibError.kOk;
    }

    public REVLibError setPositionPIDWrappingMinInput(double min) {
        if(min == positionPIDWrappingMinInput) return REVLibError.kOk;
        positionPIDWrappingMinInput = min;
        updatePIDWrapping();
        return REVLibError.kOk;
    }

    public double getP() {
        return getP(0);
    }

    public double getP(int slotID) {
        Slot slot = getSlot(slotID);
        return slot.pidController.getP();
    }

    public  MAXMotionConfig.MAXMotionPositionMode getSmartMotionAccelStrategy(int slotID) {
        return MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal;
    }

    public double getSmartMotionAllowedClosedLoopError(int slotID) {
        return getSlot(slotID).profiledPIDController.getPositionTolerance();
    }

    public double getSmartMotionMaxAccel(int slotID) {
        return getSlot(slotID).constraints.maxAcceleration;
    }

    public double getSmartMotionMaxVelocity(int slotID) {
        return getSlot(slotID).constraints.maxVelocity;
    }

    public double getSmartMotionMinOutputVelocity(int slotID) {
        return getSlot(slotID).constraints.maxAcceleration;
    }

    public REVLibError setD(double gain) {
        return setD(gain, 0);
    }

    public REVLibError setD(double gain, int slotID) {
        Slot slot = getSlot(slotID);
        slot.pidController.setD(gain);
        slot.profiledPIDController.setD(gain);
        return REVLibError.kOk;
    }

    public REVLibError setFeedbackDevice(Object sensor) {
        if (   sensor instanceof SparkRelativeEncoder
            || sensor instanceof SparkMaxAlternateEncoder
            || sensor instanceof SparkAnalogSensor
            || sensor instanceof SparkAbsoluteEncoder
            || sensor instanceof MockedEncoder) {
            if (sensor instanceof SparkRelativeEncoder) {
                SparkRelativeEncoder encoder = (SparkRelativeEncoder) sensor;
                feedbackDevice = new FeedbackDevice() {
                    @Override
                    public double getPosition() {
                        return encoder.getPosition();
                    }

                    public double getVelocity() {
                        return encoder.getVelocity();
                    }
                };
            } else if (sensor instanceof SparkMaxAlternateEncoder) {
                SparkMaxAlternateEncoder encoder = (SparkMaxAlternateEncoder) sensor;
                feedbackDevice = new FeedbackDevice() {
                    @Override
                    public double getPosition() {
                        return encoder.getPosition();
                    }

                    public double getVelocity() {
                        return encoder.getVelocity();
                    }
                };
            } else if (sensor instanceof SparkAnalogSensor) {
                SparkAnalogSensor encoder = (SparkAnalogSensor) sensor;
                feedbackDevice = new FeedbackDevice() {
                    @Override
                    public double getPosition() {
                        return encoder.getPosition();
                    }

                    public double getVelocity() {
                        return encoder.getVelocity();
                    }
                };
            } else if (sensor instanceof SparkAbsoluteEncoder) {
                SparkAbsoluteEncoder encoder = (SparkAbsoluteEncoder) sensor;
                feedbackDevice = new FeedbackDevice() {
                    @Override
                    public double getPosition() {
                        return encoder.getPosition();
                    }

                    public double getVelocity() {
                        return encoder.getVelocity();
                    }
                };
            } else {
                MockedEncoder encoder = (MockedEncoder) sensor;
                feedbackDevice = new FeedbackDevice() {
                    @Override
                    public double getPosition() {
                        return encoder.getPosition();
                    }

                    public double getVelocity() {
                        return encoder.getVelocity();
                    }
                };
            }
            return REVLibError.kOk;
        } else {
            // Right now, the SPARK MAX does not support sensors that are not directly connected to itself
            throw new IllegalArgumentException(
                sensor.getClass().getSimpleName()
                    + " cannot be used as a feedback device for a SPARK MAX at this time");
        }
    }

    public REVLibError setFF(double gain) {
        return setFF(gain, 0);
    }

    public REVLibError setFF(double gain, int slotID) {
        Slot slot = getSlot(slotID);
        slot.ff = gain;
        return REVLibError.kOk;
    }

    public REVLibError setI(double gain) {
        return setI(gain, 0);
    }

    public REVLibError setI(double gain, int slotID) {
        Slot slot = getSlot(slotID);
        slot.pidController.setI(gain);
        slot.profiledPIDController.setI(gain);
        return REVLibError.kOk;
    }

    public REVLibError setIMaxAccum(double iMaxAccum, int slotID) {
        Slot slot = getSlot(slotID);
        slot.pidController.setIntegratorRange(-iMaxAccum, iMaxAccum);
        slot.profiledPIDController.setIntegratorRange(-iMaxAccum, iMaxAccum);
        slot.iMaxAccum = iMaxAccum;
        return REVLibError.kOk;
    }

    public REVLibError setIZone(double IZone) {
        return setIZone(IZone, 0);
    }

    public REVLibError setIZone(double IZone, int slotID) {
        System.err.println("(MockedSparkMaxPIDController): setIZone() is not currently implemented");
        return REVLibError.kNotImplemented;
    }

    public REVLibError setOutputRange(double min, double max) {
        return setOutputRange(min, max, 0);
    }

    public REVLibError setOutputRange(double min, double max, int slotID) {
        Slot slot = getSlot(slotID);
        slot.outputMin = min;
        slot.outputMax = max;
        return REVLibError.kOk;
    }

    public REVLibError setP(double gain) {
        return setP(gain, 0);
    }

    public REVLibError setP(double gain, int slotID) {
        Slot slot = getSlot(slotID);
        slot.pidController.setP(gain);
        slot.profiledPIDController.setP(gain);
        return REVLibError.kOk;
    }


    public REVLibError setSmartMotionAccelStrategy(MAXMotionConfig.MAXMotionPositionMode accelStrategy, int slotID) {
        if(accelStrategy != MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal) {
            System.err.println("(MockedSparkMaxPIDController) Ignoring command to set accel strategy on slot " + slotID + " to " + accelStrategy + ". Only AccelStrategy.kTrapezoidal is supported.");
            return REVLibError.kParamNotImplementedDeprecated;
        }
        return REVLibError.kOk;
    }

    public REVLibError setSmartMotionAllowedClosedLoopError(double allowedErr, int slotId) {
        getSlot(slotId).profiledPIDController.setTolerance(allowedErr);
        return REVLibError.kOk;
    }

    public REVLibError setSmartMotionMaxAccel(double maxAccel, int slotID) {
        Slot slot = getSlot(slotID);
        slot.constraints = new TrapezoidProfile.Constraints(slot.constraints.maxVelocity, maxAccel);
        return REVLibError.kOk;
    }

    public REVLibError setSmartMotionMaxVelocity(double maxVel, int slotID) {
        Slot slot = getSlot(slotID);
        slot.constraints = new TrapezoidProfile.Constraints(maxVel, slot.constraints.maxAcceleration);
        return REVLibError.kOk;
    }

    public REVLibError setSmartMotionMinOutputVelocity(double minVel, int slotID) {
        getSlot(slotID).smartMotionMinVelocity = minVel;
        return REVLibError.kOk;
    }

    public Slot getSlot(int slotID) {
        return slots.computeIfAbsent(slotID, id -> new Slot(positionPIDWrappingMinInput, positionPIDWrappingMaxInput, positionPIDWrappingEnabled));
    }

    public static class Slot {
        private PIDController pidController = new PIDController(0, 0, 0);
        private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(0, 0);
        private ProfiledPIDController profiledPIDController = new ProfiledPIDController(0, 0, 0, constraints);
        private double ff = 0;
        private double smartMotionMinVelocity = 0;
        private double outputMin = -1;
        private double outputMax = 1;
        private double iMaxAccum = 0;

        public Slot(double positionPIDWrappingMinInput, double positionPIDWrappingMaxInput, boolean positionPIDWrappingEnabled) {
            if(positionPIDWrappingEnabled) {
                pidController.enableContinuousInput(positionPIDWrappingMinInput, positionPIDWrappingMaxInput);
                profiledPIDController.enableContinuousInput(positionPIDWrappingMinInput, positionPIDWrappingMaxInput);
            } else {
                pidController.disableContinuousInput();
                profiledPIDController.disableContinuousInput();
            }
        }
    }

    public static interface FeedbackDevice {

        public double getPosition();

        public double getVelocity();

    }

}
