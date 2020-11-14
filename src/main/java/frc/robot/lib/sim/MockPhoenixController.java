package frc.robot.lib.sim;

import java.util.HashMap;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.PWMSpeedController;

abstract class MockPhoenixController implements IMotorController {
    private final int portPWM;
    private boolean isInverted;
    // Assign the CAN port to a PWM port so it works with the simulator. Not a fan of this solution though
    // CAN ports should be separate from PWM ports
    protected PWMSpeedController motorPWM;
    // Since we need to keep a record of all the motor's followers
    protected static HashMap<Integer, PWMSpeedController> followMap = new HashMap<Integer, PWMSpeedController>();

    public MockPhoenixController(int portPWM) {
        this.portPWM = portPWM;
        isInverted = false;
    }

    public void set(double speed) {
        speed = (getInverted() ? -1.0 : 1.0) * speed;
        motorPWM.set(speed);
        if (followMap.containsKey(getDeviceID())) followMap.get(getDeviceID()).set(speed); 
    }

	public void set(ControlMode Mode, double demand) {}
    public void set(ControlMode Mode, double demand0, DemandType demand1Type, double demand1) {}
    @Deprecated
	public void set(ControlMode Mode, double demand0, double demand1) {}
	public void neutralOutput() {}
	public void setNeutralMode(NeutralMode neutralMode) {}
    public void setSensorPhase(boolean PhaseSensor) {}

    public double get() {
        return motorPWM.get();
    }

    public void follow(IMotorController leader) {
        if (!followMap.containsValue(motorPWM)) followMap.put(leader.getDeviceID(), motorPWM);
    }
    
    public void setInverted(boolean invert) { 
        isInverted = invert; 
    }

    public boolean getInverted() { 
        return isInverted; 
    }
    
	public ErrorCode configOpenloopRamp(double secondsFromNeutralToFull, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configClosedloopRamp(double secondsFromNeutralToFull, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configPeakOutputForward(double percentOut, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configPeakOutputReverse(double percentOut, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configNominalOutputForward(double percentOut, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configNominalOutputReverse(double percentOut, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configNeutralDeadband(double percentDeadband, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configVoltageCompSaturation(double voltage, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configVoltageMeasurementFilter(int filterWindowSamples, int timeoutMs) { return ErrorCode.OK; }
    
    public void enableVoltageCompensation(boolean enable) {}
    
	public double getBusVoltage() { return 0.0; }
	public double getMotorOutputPercent() { return 0.0; }
	public double getMotorOutputVoltage() { return 0.0; }
	public double getTemperature() { return 0.0; }

    public ErrorCode configSelectedFeedbackSensor(RemoteFeedbackDevice feedbackDevice, int pidIdx, int timeoutMs)  { return ErrorCode.OK; }
	public ErrorCode configSelectedFeedbackCoefficient(double coefficient, int pidIdx, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configRemoteFeedbackFilter(int deviceID, RemoteSensorSource remoteSensorSource, int remoteOrdinal,
            int timeoutMs) { return ErrorCode.OK; }
    public ErrorCode configRemoteFeedbackFilter(CANCoder canCoderRef, int remoteOrdinal, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configSensorTerm(SensorTerm sensorTerm, FeedbackDevice feedbackDevice, int timeoutMs) { return ErrorCode.OK; }

	public int getSelectedSensorPosition(int pidIdx) { return 0; }
	public int getSelectedSensorVelocity(int pidIdx) { return 0; }
    public ErrorCode setSelectedSensorPosition(int sensorPos, int pidIdx, int timeoutMs) { return ErrorCode.OK; }
    
	public ErrorCode setControlFramePeriod(ControlFrame frame, int periodMs) { return ErrorCode.OK; }
	public ErrorCode setStatusFramePeriod(StatusFrame frame, int periodMs, int timeoutMs) { return ErrorCode.OK; }
	public int getStatusFramePeriod(StatusFrame frame, int timeoutMs) { return 0; }

	public ErrorCode configForwardLimitSwitchSource(RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
            int deviceID, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configReverseLimitSwitchSource(RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
			int deviceID, int timeoutMs) { return ErrorCode.OK; }

	public void overrideLimitSwitchesEnable(boolean enable) {}

	public ErrorCode configForwardSoftLimitThreshold(int forwardSensorLimit, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configReverseSoftLimitThreshold(int reverseSensorLimit, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configForwardSoftLimitEnable(boolean enable, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configReverseSoftLimitEnable(boolean enable, int timeoutMs) { return ErrorCode.OK; }
	public void overrideSoftLimitsEnable(boolean enable) {}

	public ErrorCode config_kP(int slotIdx, double value, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode config_kI(int slotIdx, double value, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode config_kD(int slotIdx, double value, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode config_kF(int slotIdx, double value, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode config_IntegralZone(int slotIdx, int izone, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configAllowableClosedloopError(int slotIdx, int allowableCloseLoopError, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configMaxIntegralAccumulator(int slotIdx, double iaccum, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configClosedLoopPeakOutput(int slotIdx, double percentOut, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configClosedLoopPeriod(int slotIdx, int loopTimeMs, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configAuxPIDPolarity(boolean invert, int timeoutMs) { return ErrorCode.OK; }

	public ErrorCode setIntegralAccumulator(double iaccum, int pidIdx, int timeoutMs) { return ErrorCode.OK; }
	public int getClosedLoopError(int pidIdx) { return 0; }
	public double getIntegralAccumulator(int pidIdx) { return 0.0; }
	public double getErrorDerivative(int pidIdx) { return 0.0; }
	public void selectProfileSlot(int slotIdx, int pidIdx) {}
	public double getClosedLoopTarget(int pidIdx) { return 0.0; }
	public int getActiveTrajectoryPosition() { return 0; }
    public int getActiveTrajectoryVelocity() { return 0; }
    @Deprecated
	public double getActiveTrajectoryHeading() { return 0.0; }

	public ErrorCode configMotionCruiseVelocity(int sensorUnitsPer100ms, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configMotionAcceleration(int sensorUnitsPer100msPerSec, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configMotionSCurveStrength(int curveStrength, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configMotionProfileTrajectoryPeriod(int baseTrajDurationMs, int timeoutMs) { return ErrorCode.OK; }

	public ErrorCode clearMotionProfileTrajectories() { return ErrorCode.OK; }
	public int getMotionProfileTopLevelBufferCount() { return 0; }
	public ErrorCode pushMotionProfileTrajectory(TrajectoryPoint trajPt) { return ErrorCode.OK; }
	public boolean isMotionProfileTopLevelBufferFull() { return false; }
	public void processMotionProfileBuffer() {}
	public ErrorCode getMotionProfileStatus(MotionProfileStatus statusToFill) { return ErrorCode.OK; }
	public ErrorCode clearMotionProfileHasUnderrun(int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode changeMotionControlFramePeriod(int periodMs) { return ErrorCode.OK; }

	public ErrorCode getLastError() { return ErrorCode.OK; }
	public ErrorCode getFaults(Faults toFill) { return ErrorCode.OK; }
	public ErrorCode getStickyFaults(StickyFaults toFill) { return ErrorCode.OK; }
	public ErrorCode clearStickyFaults(int timeoutMs) { return ErrorCode.OK; }

	public int getFirmwareVersion() { return 0; }
	public boolean hasResetOccurred() { return false; }

	public ErrorCode configSetCustomParam(int newValue, int paramIndex, int timeoutMs) { return ErrorCode.OK; }
	public int configGetCustomParam(int paramIndex, int timeoutMs) { return 0; }

	public ErrorCode configSetParameter(ParamEnum param, double value, int subValue, int ordinal, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configSetParameter(int param, double value, int subValue, int ordinal, int timeoutMs) { return ErrorCode.OK; }
    public double configGetParameter(ParamEnum paramEnum, int ordinal, int timeoutMs) { return 0.0; }
	public double configGetParameter(int paramEnum, int ordinal, int timeoutMs) { return 0.0; }

	public int getBaseID() { return 0; }
	public int getDeviceID() { return portPWM; }
    public ControlMode getControlMode() { return ControlMode.PercentOutput; }
    
    public void valueUpdated() {}
}
