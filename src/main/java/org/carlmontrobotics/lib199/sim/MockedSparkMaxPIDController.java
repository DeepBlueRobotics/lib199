package org.carlmontrobotics.lib199.sim;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

// NOT THREAD SAFE
public class MockedSparkMaxPIDController {

    public final Map<Integer, Slot> slots = new ConcurrentHashMap<>();
    public final MockedMotorBase motor;
    public Slot activeSlot;
    public CANSparkMax.ControlType controlType = CANSparkMax.ControlType.kDutyCycle;
    public MotorController leader = null;
    public boolean invertLeader = false;
    public double setpoint = 0.0;
    public double arbFF = 0.0;
    public FeedbackDevice feedbackDevice;
    public boolean positionPIDWrappingEnabled = false;
    public double positionPIDWrappingMinInput = 0.0;
    public double positionPIDWrappingMaxInput = 0.0;

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
            case kSmartMotion:
                output = activeSlot.profiledPIDController.calculate(feedbackDevice.getPosition(), setpoint);
                if(Math.abs(activeSlot.profiledPIDController.getGoal().velocity) < activeSlot.smartMotionMinVelocity) {
                    output = 0;
                }
                break;
            case kCurrent:
                output = activeSlot.pidController.calculate(currentDraw, setpoint);
                break;
            case kSmartVelocity:
            default:
                throw new IllegalArgumentException("Unsupported ControlType: " + controlType);
        }
        output += activeSlot.ff * setpoint + arbFF;
        output = MathUtil.clamp(output, activeSlot.outputMin, activeSlot.outputMax);
        return output;
    }

    public void setDutyCycle(double speed) {
        setpoint = speed;
        controlType = CANSparkMax.ControlType.kDutyCycle;
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

    public MockedSparkMaxPIDController(MockedMotorBase motor) {
        this.motor = motor;
        slots.put(0, activeSlot = new Slot(positionPIDWrappingMinInput, positionPIDWrappingMaxInput, positionPIDWrappingEnabled));
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

    public double getIAccum() {
        System.err.println("(MockedSparkMaxPIDController): getIAccum() is not currently implemented");
        return 0;
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

    public REVLibError setPositionPIDWrappingEnable(boolean enable) {
        if(enable == positionPIDWrappingEnabled) return REVLibError.kOk;
        positionPIDWrappingEnabled = enable;
        slots.values().forEach(slot -> {
            if(enable) {
                slot.pidController.enableContinuousInput(positionPIDWrappingMinInput, positionPIDWrappingMaxInput);
                slot.profiledPIDController.enableContinuousInput(positionPIDWrappingMinInput, positionPIDWrappingMaxInput);
            } else {
                slot.pidController.disableContinuousInput();
                slot.profiledPIDController.disableContinuousInput();
            }
        });
        return REVLibError.kOk;
    }

    public REVLibError setPositionPIDWrappingMaxInput(double max) {
        if(max == positionPIDWrappingMaxInput) return REVLibError.kOk;
        positionPIDWrappingMaxInput = max;
        slots.values().forEach(slot -> {
            slot.pidController.enableContinuousInput(positionPIDWrappingMinInput, positionPIDWrappingMaxInput);
            slot.profiledPIDController.enableContinuousInput(positionPIDWrappingMinInput, positionPIDWrappingMaxInput);
        });
        return REVLibError.kOk;
    }

    public REVLibError setPositionPIDWrappingMinInput(double min) {
        if(min == positionPIDWrappingMinInput) return REVLibError.kOk;
        positionPIDWrappingMinInput = min;
        slots.values().forEach(slot -> {
            slot.pidController.enableContinuousInput(positionPIDWrappingMinInput, positionPIDWrappingMaxInput);
            slot.profiledPIDController.enableContinuousInput(positionPIDWrappingMinInput, positionPIDWrappingMaxInput);
        });
        return REVLibError.kOk;
    }

    public double getP() {
        return getP(0);
    }

    public double getP(int slotID) {
        Slot slot = getSlot(slotID);
        return slot.pidController.getP();
    }

    public SparkMaxPIDController.AccelStrategy getSmartMotionAccelStrategy(int slotID) {
        return SparkMaxPIDController.AccelStrategy.kTrapezoidal;
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

    public REVLibError setFeedbackDevice(MotorFeedbackSensor sensor) {
        if (sensor instanceof SparkMaxRelativeEncoder
            || sensor instanceof SparkMaxAlternateEncoder
            || sensor instanceof SparkMaxAnalogSensor
            || sensor instanceof SparkMaxAbsoluteEncoder
            || sensor instanceof MockedEncoder) {
            if (sensor instanceof SparkMaxRelativeEncoder) {
                SparkMaxRelativeEncoder encoder = (SparkMaxRelativeEncoder) sensor;
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
            } else if (sensor instanceof SparkMaxAnalogSensor) {
                SparkMaxAnalogSensor encoder = (SparkMaxAnalogSensor) sensor;
                feedbackDevice = new FeedbackDevice() {
                    @Override
                    public double getPosition() {
                        return encoder.getPosition();
                    }

                    public double getVelocity() {
                        return encoder.getVelocity();
                    }
                };
            } else if (sensor instanceof SparkMaxAbsoluteEncoder) {
                SparkMaxAbsoluteEncoder encoder = (SparkMaxAbsoluteEncoder) sensor;
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

    public REVLibError setIAccum(double iAccum) {
        System.err.println("(MockedSparkMaxPIDController): setIAccum() is not currently implemented");
        return REVLibError.kNotImplemented;
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

    public REVLibError setReference(double value, CANSparkMax.ControlType ctrl) {
        return setReference(value, ctrl, 0);
    }

    public REVLibError setReference(double value, CANSparkMax.ControlType ctrl, int pidSlot) {
        return setReference(value, ctrl, pidSlot, 0);
    }

    public REVLibError setReference(double value, CANSparkMax.ControlType ctrl, int pidSlot, double arbFeedforward) {
        return setReference(value, ctrl, pidSlot, arbFeedforward, SparkMaxPIDController.ArbFFUnits.kVoltage);
    }

    public REVLibError setReference(double value, CANSparkMax.ControlType ctrl, int pidSlot, double arbFeedforward, SparkMaxPIDController.ArbFFUnits arbFFUnits) {
        if(ctrl == CANSparkMax.ControlType.kSmartVelocity) {
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
            case kCurrent:
                motor.setClosedLoopControl(true);
                break;
            case kSmartVelocity:
                break; // This should never happen
        }

        activeSlot = getSlot(pidSlot);

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

    public REVLibError setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy accelStrategy, int slotID) {
        if(accelStrategy != SparkMaxPIDController.AccelStrategy.kTrapezoidal) {
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
