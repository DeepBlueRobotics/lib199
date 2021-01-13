package frc.robot.lib.sim;

import java.util.ArrayList;
import java.util.HashMap;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.PWMSpeedController;
import frc.robot.lib.Mocks;

abstract class MockPhoenixController implements IMotorController {
    private final int portPWM;
	private boolean isInverted;
	private SensorCollection sensorCollection;
	private FeedbackDevice feedbackDevice;
    // Assign the CAN port to a PWM port so it works with the simulator. Not a fan of this solution though
    // CAN ports should be separate from PWM ports
	protected PWMSpeedController motorPWM;
    // Since we need to keep a record of all the motor's followers
	protected static HashMap<Integer, ArrayList<PWMSpeedController>> followMap = new HashMap<>();

    public MockPhoenixController(int portPWM) {
        this.portPWM = portPWM;
		isInverted = false;
		sensorCollection = Mocks.createMock(SensorCollection.class, new MockSensorCollection(portPWM));
		feedbackDevice = FeedbackDevice.QuadEncoder;
    }

    public void set(double speed) {
        speed = (getInverted() ? -1.0 : 1.0) * speed;
        motorPWM.set(speed);
        if (followMap.containsKey(getDeviceID())) {
            for (PWMSpeedController motor : followMap.get(getDeviceID())) motor.set(speed);
        }
    }

	public void set(ControlMode Mode, double demand) {
		set(Mode, demand, DemandType.Neutral, 0);
	}

	@Deprecated
	public void set(ControlMode Mode, double demand0, double demand1) {
		// Ignore demand1
		set(Mode, demand0, DemandType.Neutral, 0);
	}

    public void set(ControlMode Mode, double demand0, DemandType demand1Type, double demand1) {
		// Ignore demand1Type and demand1

		switch (Mode) {
			case PercentOutput:
				set(demand0);
				break;
			case Position:
			case Velocity:
			case Current:
			case Follower:
			case MotionProfile:
			case MotionMagic:
			case MotionProfileArc:
			case MusicTone:
			case Disabled:
			default:
				break;
		}
	}

    public double get() {
        return motorPWM.get();
	}
	
	public double getMotorOutputPercent() { return get(); }

    public void follow(IMotorController leader) {
        if (!followMap.containsKey(leader.getDeviceID())) {
            ArrayList<PWMSpeedController> arr = new ArrayList<PWMSpeedController>();
            arr.add(motorPWM);
            followMap.put(leader.getDeviceID(), arr);
        } else {
            followMap.get(leader.getDeviceID()).add(motorPWM);
        }
    }
    
    public void setInverted(boolean invert) { 
        isInverted = invert; 
    }

    public boolean getInverted() {
        return isInverted;
	}
	
	public int getSelectedSensorPosition(int pidIdx) {
		switch (feedbackDevice) {
			case QuadEncoder:
				return sensorCollection.getQuadraturePosition();
			case Analog:
				return sensorCollection.getAnalogIn();
			case PulseWidthEncodedPosition:
				return sensorCollection.getPulseWidthPosition();
			case IntegratedSensor:
			case Tachometer:
			case SensorSum:
			case SensorDifference:
			case RemoteSensor0:
			case RemoteSensor1:
			case SoftwareEmulatedSensor:
			case CTRE_MagEncoder_Absolute:
			case CTRE_MagEncoder_Relative:
			case None:
			default:
				return 0;
		}
	}

	public int getSelectedSensorVelocity(int pidIdx) {
		switch (feedbackDevice) {
			case QuadEncoder:
				return sensorCollection.getQuadratureVelocity();
			case Analog:
				return sensorCollection.getAnalogInVel();
			case PulseWidthEncodedPosition:
				return sensorCollection.getPulseWidthVelocity();
			case IntegratedSensor:
			case Tachometer:
			case SensorSum:
			case SensorDifference:
			case RemoteSensor0:
			case RemoteSensor1:
			case SoftwareEmulatedSensor:
			case CTRE_MagEncoder_Absolute:
			case CTRE_MagEncoder_Relative:
			case None:
			default:
				return 0;
		}
	}

	public ErrorCode setSelectedSensorPosition(int sensorPos) { return setSelectedSensorPosition(sensorPos, 0, 0); }

	public ErrorCode setSelectedSensorPosition(int sensorPos, int pidIdx, int timeoutMs) {
		switch (feedbackDevice) {
			case QuadEncoder:
				break;
			case Analog:
				break;
			case PulseWidthEncodedPosition:
			case IntegratedSensor:
			case Tachometer:
			case SensorSum:
			case SensorDifference:
			case RemoteSensor0:
			case RemoteSensor1:
			case SoftwareEmulatedSensor:
			case CTRE_MagEncoder_Absolute:
			case CTRE_MagEncoder_Relative:
			case None:
			default:
				break;
		}
		return ErrorCode.OK;
	}

	public ErrorCode configSelectedFeedbackSensor(FeedbackDevice feedbackDevice) {
		this.feedbackDevice = feedbackDevice;
		return ErrorCode.OK;
	}

	public SensorCollection getSensorCollection() {
		return sensorCollection;
	}

	public int getDeviceID() {
		return portPWM;
	}
	
	/*
	#############
	METHOD STUBS
	#############
	*/

	public void neutralOutput() {}
	public void setNeutralMode(NeutralMode neutralMode) {}
    public void setSensorPhase(boolean PhaseSensor) {}

	public ErrorCode configOpenloopRamp(double secondsFromNeutralToFull, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configClosedloopRamp(double secondsFromNeutralToFull, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configPeakOutputForward(double percentOut, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configPeakOutputReverse(double percentOut, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configNominalOutputForward(double percentOut, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configNominalOutputReverse(double percentOut, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configNeutralDeadband(double percentDeadband, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configVoltageCompSaturation(double voltage, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configVoltageMeasurementFilter(int filterWindowSamples, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configPeakCurrentLimit(int amps, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configPeakCurrentDuration(int milliseconds, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configContinuousCurrentLimit(int amps, int timeoutMs) { return ErrorCode.OK; }
    
	public void enableVoltageCompensation(boolean enable) {}
	public void enableCurrentLimit(boolean enable) {}
    
	public double getBusVoltage() { return 0.0; }
	public double getMotorOutputVoltage() { return 0.0; }
	public double getTemperature() { return 0.0; }

	public ErrorCode configSelectedFeedbackSensor(RemoteFeedbackDevice feedbackDevice, int pidIdx, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configSelectedFeedbackCoefficient(double coefficient, int pidIdx, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configRemoteFeedbackFilter(int deviceID, RemoteSensorSource remoteSensorSource, int remoteOrdinal,
            int timeoutMs) { return ErrorCode.OK; }
    public ErrorCode configRemoteFeedbackFilter(CANCoder canCoderRef, int remoteOrdinal, int timeoutMs) { return ErrorCode.OK; }
	public ErrorCode configSensorTerm(SensorTerm sensorTerm, FeedbackDevice feedbackDevice, int timeoutMs) { return ErrorCode.OK; }
    
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
    public ControlMode getControlMode() { return ControlMode.PercentOutput; }
    
    public void valueUpdated() {}
}
