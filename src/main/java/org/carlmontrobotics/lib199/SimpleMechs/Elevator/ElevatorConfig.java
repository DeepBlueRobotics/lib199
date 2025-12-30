package org.carlmontrobotics.lib199.SimpleMechs.Elevator;
import java.util.function.BooleanSupplier;

import org.carlmontrobotics.lib199.MotorConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;

public final class ElevatorConfig {
    public final int elevatorMasterId;
    public final int elevatorFollowId; //make your own config if you have more than 2 motors
    public final MotorConfig elevatorMasterMotorType;
    public final MotorConfig elevatorFollowMotorType;
    public final boolean elevatorMasterInverted;
    public final boolean elevatorFollowInverted;
    public final BooleanSupplier bottomReset;
    public final BooleanSupplier topReset;
    public final double topLimit; //meters
    public final double bottomLimit; //meters
    public final double gearReduction; //rotations to meters
    public double[] elevatorKPID;
    public double[] elevatorKFeedForward;
    public double elevatorPIDTolerance;
    public double maxVolts; //14 for no limit
    public double maxManualInput; //percentage of power

    public double elevatorKP;
    public double elevatorKI;
    public double elevatorKD;
    public double elevatorKS;
    public double elevatorKG;
    public double elevatorKV;
    public double elevatorKA;

    public boolean elevatorPIDExists;
    public boolean elevatorFeedForwardExists;

    public PIDController elevatorPID;
    public ElevatorFeedforward elevatorFeedforward;


    /**
     * Creates a elevator config for a {@link SimpleElevator}
     * @param elevatorMasterId the port id of the master motor, int
     * @param elevatorMasterMotorType the {@link MotorConfig} of the master motor, that is NEO_550, NEO, and NEO_VORTEX
     * @param elevatorMasterInverted setting for positive direction of the master motor would make the elevator go up, boolean
     * @param elevatorFollowId the port id for the follow motor, int
     * @param elevatorFollowMotorType the {@link MotorConfig} of the follow motor, that is NEO_550, NEO, and NEO_VORTEX, generally should be the same as the master
     * @param elevatorFollowInverted setting for positive direction of teh follow motor would make the elevator go up, boolean
     * @param bottomReset a {@link BooleanSupplier}, that would be true if the elevator is fully down (Limit switch), null if none
     * @param topReset a {@link BooleanSupplier}, that would be true if the elevator is fully up(Limit switch), null if none
     * @param bottomLimit the lowest value possible the elevator can achieve based of encoder readings in meters, generally 0
     * @param topLimit the highest value possible the elevator can achieve based of encoder readings in meters
     * @param elevatorKPID double array of [kP, kI, kD] have to be >= 0
     * @param elevatorKFeedForward double array of [kS, kG, kV, kA] all but kG have to be >= 0
     * @param gearReduction # of rotations of the motor to raise the elevator 1 meter (Elevator must have linear increase in height)
     * @param elevatorPIDTolerance double, how many METERS of tolerance is the user fine with
     * @param maxVolts maximum volts to use on the motors, 14 if no limit
     * @param maxManualInput maximum percentage of how much voltage a user can use from 0-1.0
     * @throws IllegalArgumentException
     * @throws IllegalArgumentException
     * @throws IllegalArgumentException
     */
    public ElevatorConfig(int elevatorMasterId, MotorConfig elevatorMasterMotorType, boolean elevatorMasterInverted,
                          int elevatorFollowId, MotorConfig elevatorFollowMotorType, boolean elevatorFollowInverted,
                          BooleanSupplier bottomReset, BooleanSupplier topReset, double bottomLimit, double topLimit,
                          double[] elevatorKPID, double[] elevatorKFeedForward,
                          double gearReduction, double elevatorPIDTolerance, double maxVolts, double maxManualInput) {
        
        this.elevatorMasterId = elevatorMasterId;
        this.elevatorMasterMotorType = elevatorMasterMotorType;
        this.elevatorMasterInverted = elevatorMasterInverted;
        this.elevatorFollowId = elevatorFollowId;
        this.elevatorFollowMotorType = elevatorFollowMotorType;
        this.elevatorFollowInverted = elevatorFollowInverted;
        this.bottomReset = bottomReset;
        this.topReset = topReset;
        this.bottomLimit = bottomLimit;
        this.topLimit = topLimit;
        this.gearReduction = gearReduction;
        this.elevatorPIDTolerance = elevatorPIDTolerance;
        this.maxVolts = maxVolts;
        this.maxManualInput = maxManualInput;
        this.elevatorKPID = elevatorKPID;
        this.elevatorKFeedForward = elevatorKFeedForward;
        checkRequirements();
    }

    public static ElevatorConfig NEOElevatorConfig(int elevatorMasterId, boolean elevatorMasterInverted, int elevatorFollowId, boolean elevatorFollowInverted, BooleanSupplier bottomReset, BooleanSupplier topReset, double topLimit, double[] elevatorKPID, double[] elevatorKFeedForward, double gearReduction, double maxManualInput)  {
        return new ElevatorConfig(elevatorMasterId, MotorConfig.NEO, elevatorMasterInverted, elevatorFollowId, MotorConfig.NEO, elevatorFollowInverted, bottomReset, topReset, 0, topLimit, elevatorKPID, elevatorKFeedForward, gearReduction, 0.02, 14, maxManualInput);
    }

    public static ElevatorConfig VortexElevatorConfig(int elevatorMasterId, boolean elevatorMasterInverted, int elevatorFollowId, boolean elevatorFollowInverted, BooleanSupplier bottomReset, BooleanSupplier topReset, double topLimit, double[] elevatorKPID, double[] elevatorKFeedForward, double gearReduction, double maxManualInput)  {
        return new ElevatorConfig(elevatorMasterId, MotorConfig.NEO_VORTEX, elevatorMasterInverted, elevatorFollowId, MotorConfig.NEO_VORTEX, elevatorFollowInverted, bottomReset, topReset, 0, topLimit, elevatorKPID, elevatorKFeedForward, gearReduction, 0.02, 14, maxManualInput);
    }

    /**
     * Checks the elevatorConfig to not have any issues, and creates certain extra values automatically like {@link #elevatorPID} and {@link #elevatorFeedforward}
     */
    private void checkRequirements() {
        checkMotorConfigs(); //Prevent future issues with new motors
        checkPID();
        checkFeedForward();
        checkGearReduction();
        checkLimits();
    }

    /**
     * Check that the motor Configs are not any new configs that would not work or null.
     */
    private void checkMotorConfigs() {
        if (elevatorFollowMotorType != MotorConfig.NEO && elevatorFollowMotorType != MotorConfig.NEO_VORTEX && elevatorFollowMotorType != MotorConfig.NEO_550) {
            throw new IllegalArgumentException("What is this config??? If null pls change, if not null you're cooked");
        }
        if (elevatorMasterMotorType != MotorConfig.NEO && elevatorMasterMotorType != MotorConfig.NEO_VORTEX && elevatorMasterMotorType != MotorConfig.NEO_550) {
            throw new IllegalArgumentException("What is this config??? If null pls change, if not null you're cooked");
        }
    }

    /**
     * Checks PID to have 3 values, updates {@link #elevatorPIDExists} and makes sure the values are legitimate
     */
    private void checkPID() {
        if (elevatorKPID != null) {
            if (elevatorKPID.length == 3) {
                if (elevatorKPID[0] < 0 || elevatorKPID[1] < 0 || elevatorKPID[2] < 0) {
                    throw new IllegalArgumentException("PID values have to be non negative");
                }
                elevatorKP = elevatorKPID[0];
                elevatorKI = elevatorKPID[1];
                elevatorKD = elevatorKPID[2];
                elevatorPID = new PIDController(elevatorKP, elevatorKI, elevatorKD);
                elevatorPID.setTolerance(elevatorPIDTolerance);
                elevatorPIDExists = true;
                return;
            }
            throw new IllegalArgumentException("Need to have 3 values for PID");
        }
        elevatorKP = 0;
        elevatorKI = 0;
        elevatorKD = 0;
        elevatorPIDExists = false;
        DriverStation.reportWarning("Elevator PID is off", true);
    }

    /**
     * Checks FeedForward to have 4 values, updates {@link #elevatorFeedForwardExists} and makes sure the values are legitimate
     */
    private void checkFeedForward() {
        if (elevatorKFeedForward != null) {
            if (elevatorKFeedForward.length == 4) {
                if (elevatorKFeedForward[0] < 0|| elevatorKFeedForward[2] < 0 || elevatorKFeedForward[3] < 0) {
                    throw new IllegalArgumentException("FeedForward Values of kS, kV, and kA need to have non negative values");
                }
                elevatorKS = elevatorKFeedForward[0];
                elevatorKG = elevatorKFeedForward[1];
                elevatorKV = elevatorKFeedForward[2];
                elevatorKA = elevatorKFeedForward[3];
                elevatorFeedforward = new ElevatorFeedforward(elevatorKS, elevatorKG, elevatorKV, elevatorKA);
                elevatorFeedForwardExists = true;
                return;
            }
            throw new IllegalArgumentException("Need to have 4 values for Elevator FeedForward: Friction, Gravity, Voltage, and Acceleration");
        }
        elevatorKS = 0;
        elevatorKG = 0;
        elevatorKV = 0;
        elevatorKA = 0;
        elevatorFeedForwardExists = false;
        DriverStation.reportWarning("ElevatorFeedForward is off", true);
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
     * Makes sure that the user has not mixed up the top and bottom limits.
     */
    private void checkLimits() {
        if (bottomLimit >= topLimit) {
            throw new IllegalArgumentException("Top limit must be much greater than bottom limit. (Cannot be equal)");
        }
    }
}