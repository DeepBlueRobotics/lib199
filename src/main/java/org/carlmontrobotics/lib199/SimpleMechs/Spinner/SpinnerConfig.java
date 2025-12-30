package org.carlmontrobotics.lib199.SimpleMechs.Spinner;

import org.carlmontrobotics.lib199.MotorConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;

public final class SpinnerConfig {
    public final int spinnerMasterId;
    public final int spinnerFollowId;
    public final MotorConfig spinnerMasterMotorType;
    public final MotorConfig spinnerFollowMotorType;
    public final boolean spinnerMasterInverted;
    public final boolean spinnerFollowInverted;
    public final double gearReduction;
    public final double maxVolts;
    public final double maxManualInput;
    public final double[] spinnerKPID;
    public final double[] spinnerKFeedForward;
    public final double spinnerPIDTolerance;
    public double spinnerKP;
    public double spinnerKI;
    public double spinnerKD;
    public double spinnerKS;
    public double spinnerKV;
    public double spinnerKA;
    public PIDController spinnerPID;
    public SimpleMotorFeedforward spinnerFeedforward;
    public boolean spinnerFollowExists;
    public boolean spinnerPIDExists;
    public boolean spinnerFeedForwardExists;

    /**
     * 
     * @param spinnerMasterId master id
     * @param spinnerFollowId follow id, -1 if no follow motor
     * @param spinnerMasterMotorType master {@link MotorConfig}
     * @param spinnerFollowMotorType follow {@link MotorConfig} null if no follow motor
     * @param spinnerMasterInverted master inversed such that whichever is the desired movement of the spinner will be positive
     * @param spinnerFollowInverted follow inversed such that whichever is the desired movement of the spinner will be positive, false if no motor
     * @param gearReduction 1 revolution of the spinner / how many revolution is needed by the motor
     * @param maxVolts maximum volts that the spinner can use autonomously
     * @param maxManualInput maximum voltage percentage that the spinner can use while being manually controlled
     * @param spinnerKPID 3 values for kP, kI, and kD
     * @param spinnerKFeedForward 3 values for kS, kV, and kA
     * @param spinnerPIDTolerance in rotations per second how much should pid care for speed
     * @throws IllegalArgumentException if SpinnerFollowMotorType is null when there is provided a spinnerFollowId
     * @throws IllegalArgumentException if SpinnerFollowMotorType or SpinnerMasterMotorType is not an expected {@link MotorConfig}
     * @throws IllegalArgumentException if spinnerKPID is not three values with each being non negative
     * @throws IllegalArgumentException if spinnerKFeedForward is not three values with each being non negative
     * @throws IllegalArgumentException if gearReduction is 0
     * @throws IllegalArgumentException if maxVolts is not positive
     * @throws IllegalArgumentException if maxManualInput is not positive
     */
    public SpinnerConfig(int spinnerMasterId, int spinnerFollowId, MotorConfig spinnerMasterMotorType, MotorConfig spinnerFollowMotorType, boolean spinnerMasterInverted, boolean spinnerFollowInverted, 
                        double gearReduction, double maxVolts, double maxManualInput, double[] spinnerKPID, double[] spinnerKFeedForward, double spinnerPIDTolerance) {
        this.spinnerMasterId = spinnerMasterId;
        this.spinnerMasterMotorType = spinnerMasterMotorType;
        this.spinnerMasterInverted = spinnerMasterInverted;
        this.spinnerFollowId = spinnerFollowId;
        this.spinnerFollowMotorType = spinnerFollowMotorType;
        this.spinnerFollowInverted = spinnerFollowInverted;
        this.gearReduction = gearReduction;
        this.maxVolts = maxVolts;
        this.maxManualInput = maxManualInput;
        this.spinnerKPID = spinnerKPID;
        this.spinnerKFeedForward = spinnerKFeedForward;
        this.spinnerPIDTolerance = spinnerPIDTolerance;
        checkRequirements();
    }

    /**
     * Creates a simple single NEO spinner with PID
     * @param spinnerMasterId id of the motor
     * @param spinnerMasterInverted inversed setting such that whichever is the desired movement of the spinner will be positive
     * @param gearReduction 1 revolution of the spinner / how many revolution is needed by the motor
     * @param maxVolts maximum volts that the spinner can use autonomously
     * @param spinnerKPID 3 values for kP, kI, and kD
     * @param spinnerKPIDTolerance in rotations per second how much should pid care for speed
     */
    public static SpinnerConfig NEOSpinnerConfig(int spinnerMasterId, boolean spinnerMasterInverted, double gearReduction, double maxVolts, double[] spinnerKPID, double spinnerPIDTolerance) {
        return new SpinnerConfig(spinnerMasterId, -1, MotorConfig.NEO, null, spinnerMasterInverted, false, gearReduction, maxVolts, maxVolts/12.0, spinnerKPID, null, spinnerPIDTolerance);
    }

    /**
     * Creates a simple single NEO spinner without PID
     * @param spinnerMasterId id of the motor
     * @param spinnerMasterInversed inversed setting such that whichever is the desired movement of the spinner will be positive
     * @param gearReduction 1 revolution of the spinner / how many revolution is needed by the motor
     * @param maxManualInput maximum voltage percentage that the spinner can use while being manually controlled
     * @return
     */
    public static SpinnerConfig NEOSpinnerConfig(int spinnerMasterId, boolean spinnerMasterInverted, double gearReduction, double maxManualInput) {
        return new SpinnerConfig(spinnerMasterId, -1, MotorConfig.NEO, null, spinnerMasterInverted, false, gearReduction, 14, maxManualInput, null, null, 0);
    }

    /**
     * Creates a simple single NEO spinner with PID
     * @param spinnerMasterId id of the motor
     * @param spinnerMasterInversed inversed setting such that whichever is the desired movement of the spinner will be positive
     * @param gearReduction 1 revolution of the spinner / how many revolution is needed by the motor
     * @param maxVolts maximum volts that the spinner can use autonomously
     * @param spinnerKPID 3 values for kP, kI, and kD
     * @param spinnerKPIDTolerance in rotations per second how much should pid care for speed
     */
    public static SpinnerConfig VORTEXSpinnerConfig(int spinnerMasterId, boolean spinnerMasterInverted, double gearReduction, double maxVolts, double[] spinnerKPID, double spinnerPIDTolerance) {
        return new SpinnerConfig(spinnerMasterId, -1, MotorConfig.NEO_VORTEX, null, spinnerMasterInverted, false, gearReduction, maxVolts, maxVolts/12.0, spinnerKPID, null, spinnerPIDTolerance);
    }

    /**
     * Creates a simple single NEO spinner without PID
     * @param spinnerMasterId id of the motor
     * @param spinnerMasterInversed inversed setting such that whichever is the desired movement of the spinner will be positive
     * @param gearReduction 1 revolution of the spinner / how many revolution is needed by the motor
     * @param maxManualInput maximum voltage percentage that the spinner can use while being manually controlled
     * @return
     */
    public static SpinnerConfig VORTEXSpinnerConfig(int spinnerMasterId, boolean spinnerMasterInverted, double gearReduction, double maxManualInput) {
        return new SpinnerConfig(spinnerMasterId, -1, MotorConfig.NEO_VORTEX, null, spinnerMasterInverted, false, gearReduction, 14, maxManualInput, null, null, 0);
    }

    /**
     * Checks the spinnerConfig to not have any issues, and creates certain extra values automatically like {@link #spinnerPID} and {@link #spinnerFeedforward}
     */
    private void checkRequirements() {
        checkMotorConfigs(); //Prevent future issues with new motors
        checkPID();
        checkFeedForward();
        checkGearReduction();
        checkMaxVoltsAndInput();
    }

    /**
     * Check that the motor Configs are not any new configs that would not work or null.
     */
    private void checkMotorConfigs() {
        if (spinnerFollowId != -1) {
            spinnerFollowExists = true;
            if (spinnerFollowMotorType != MotorConfig.NEO && spinnerFollowMotorType != MotorConfig.NEO_VORTEX && spinnerFollowMotorType != MotorConfig.NEO_550) {
                throw new IllegalArgumentException("What is this config??? If null pls change, if not null you're cooked");
            }
        }
        else {
            spinnerFollowExists = false;
        }
        if (spinnerMasterMotorType != MotorConfig.NEO && spinnerMasterMotorType != MotorConfig.NEO_VORTEX && spinnerMasterMotorType != MotorConfig.NEO_550) {
            throw new IllegalArgumentException("What is this config??? If null pls change, if not null you're cooked");
        }
    }

    /**
     * Checks PID to have 3 values, updates {@link #spinnerPIDExists} and makes sure the values are legitimate
     */
    private void checkPID() {
        if (spinnerKPID != null) {
            if (spinnerKPID.length == 3) {
                if (spinnerKPID[0] < 0 || spinnerKPID[1] < 0 || spinnerKPID[2] < 0) {
                    throw new IllegalArgumentException("PID values have to be non negative");
                }
                spinnerKP = spinnerKPID[0];
                spinnerKI = spinnerKPID[1];
                spinnerKD = spinnerKPID[2];
                spinnerPID = new PIDController(spinnerKP, spinnerKI, spinnerKD);
                spinnerPID.setTolerance(spinnerPIDTolerance);
                spinnerPID.enableContinuousInput(0, 1);
                spinnerPIDExists = true;
                return;
            }
            throw new IllegalArgumentException("Need to have 3 values for PID");
        }
        spinnerKP = 0;
        spinnerKI = 0;
        spinnerKD = 0;
        spinnerPIDExists = false;
        DriverStation.reportWarning("Spinner PID is off", true);
    }

    /**
     * Checks FeedForward to have 3 values, updates {@link #spinnerFeedForwardExists} and makes sure the values are legitimate
     */
    private void checkFeedForward() {
        if (spinnerKFeedForward != null) {
            if (spinnerKFeedForward.length == 3) {
                if (spinnerKFeedForward[0] < 0|| spinnerKFeedForward[1] < 0 || spinnerKFeedForward[2] < 0) {
                    throw new IllegalArgumentException("FeedForward values need to have non negative values");
                }
                spinnerKS = spinnerKFeedForward[0];
                spinnerKV = spinnerKFeedForward[1];
                spinnerKA = spinnerKFeedForward[2];
                spinnerFeedforward = new SimpleMotorFeedforward(spinnerKS, spinnerKV, spinnerKA);
                spinnerFeedForwardExists = true;
                return;
            }
            throw new IllegalArgumentException("Need to have 3 values for Spinner FeedForward: Friction, Voltage, and Acceleration");
        }
        spinnerKS = 0;
        spinnerKV = 0;
        spinnerKA = 0;
        spinnerFeedForwardExists = false;
        DriverStation.reportWarning("SpinnerFeedForward is off", true);
    }   

    /**
     * Makes sure gear reduction is not 0
     */
    private void checkGearReduction() {
        if (gearReduction == 0) {
            throw new IllegalArgumentException("Gear reduction cannot be 0");
        }
    }

    /**
     * Checks for {@link #maxVolts} and {@link #maxManualInput} to be greater than zero
     */
    private void checkMaxVoltsAndInput() {
        if (maxVolts <= 0) {
            throw new IllegalArgumentException("maxVolts needs to be positive");
        }
        if (maxManualInput <= 0) {
            throw new IllegalArgumentException("maxManualInput needs to be positive");
        }
    }
}