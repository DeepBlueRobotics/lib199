package org.carlmontrobotics.lib199.SimpleMechs.Elevator;
import java.util.function.BooleanSupplier;

import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import edu.wpi.first.math.controller.ArmFeedforward;
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



    public ElevatorConfig(int elevatorMasterId, MotorConfig elevatorMasterMotorType, boolean elevatorMasterInverted,
                          int elevatorFollowId, MotorConfig elevatorFolllowMotorType, boolean elevatorFollowInverted,
                          BooleanSupplier bottomReset, BooleanSupplier topReset, double bottomLimit, double topLimit,
                          double[] elevatorKPID, double[] elevatorKFeedForward,
                          double gearReduction, double elevatorPIDTolerance, double maxVolts, double maxManualInput) {
        
        this.elevatorMasterId = elevatorMasterId;
        this.elevatorMasterMotorType = elevatorMasterMotorType;
        this.elevatorMasterInverted = elevatorMasterInverted;
        this.elevatorFollowId = elevatorFollowId;
        this.elevatorFollowMotorType = elevatorFolllowMotorType;
        this.elevatorFollowInverted = elevatorFollowInverted;
        this.bottomReset = bottomReset;
        this.topReset = topReset;
        this.bottomLimit = bottomLimit;
        this.topLimit = topLimit;
        this.gearReduction = gearReduction;
        this.elevatorPIDTolerance = elevatorPIDTolerance;
        this.maxVolts = maxVolts;
        this.maxManualInput = maxManualInput;
        checkRequirements();
    }

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


    private void checkPID() {
        if (elevatorKPID != null) {
            if (elevatorKPID.length == 3) {
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

    private void checkFeedForward() {
        if (elevatorKFeedForward != null) {
            if (elevatorKFeedForward.length == 4) {
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

    private void checkGearReduction() {
        if (gearReduction == 0) {
            throw new IllegalArgumentException("Gear reduction cannot be 0");
        }
    }

    private void checkLimits() {
        if (bottomLimit >= topLimit) {
            throw new IllegalArgumentException("Top limit must be much greater than bottom limit. (Cannot be equal)");
        }
    }
}