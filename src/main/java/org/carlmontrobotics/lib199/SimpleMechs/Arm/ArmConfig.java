package org.carlmontrobotics.lib199.SimpleMechs.Arm;
import org.carlmontrobotics.lib199.MotorConfig;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;


public final class ArmConfig {
    public final double armRelativeGearReduction, armAbsoluteGearReduction, armPIDTolerance, armMaxVolts;
    public final int[] armFollowId;
    public final MotorConfig[] armFollowMotorType; 
    public final boolean[] armFollowInversed;

    public final int armMasterMotorId;
    public final MotorConfig armMasterMotorType; 
    public final boolean armMasterInversed;
    
    public final double [] armKPID, armKFeedForward;
    public double armKP, armKI, armKD, armKS, armKG, armKV, armKA;
    
    public AbsoluteEncoder armMainAbsoluteEncoder;

    public ArmFeedforward armFeedForward;
    public PIDController armPID;
    public boolean armPIDContinuousInput;

    public double armAbsoluteZeroOffset;
    public double bottomLimit;
    public double topLimit;

    public int armMotorOfBackupRelativeEncoderId;


    /**
     * Creates a ArmConfig for a SimpleArm
     * @param armRelativeGearReduction Gear reduction from shaft spinning where motors are to the arm axis
     * @param armAbsoluteGearReduction Gear reduction from absolute encoder reading to the arm axis
     * @param armMaxVolts Max volts that motors can output on to the arm, 14 if no limit
     * @param armFollowId List all motorIds for followers
     * @param armFollowMotorType List all motor configs for the followers in the same order as above
     * @param armFollowInversed List all inverse configs for the followers in the same order as above
     * @param armMasterMotorId Master id
     * @param armMasterMotorType Motor Config of Master
     * @param armMasterInversed Inverse Config of Master
     * @param armKPID (kP, kI, kD) of arm
     * @param armPIDTolerance Tolerance (in degrees) of arm
     * @param armPIDContinuousInput True if arm can do infinite 360s, keep false otherwise!
     * @param armKFeedForward (kS, kG, kV, kA) of arm
     * @param armMainAbsoluteEncoder null if no absolute
     * @param armMotorIDOfBackupRelativeEncoder -1 if none
     * @param armAbsoluteZeroOffset zeroOffset (in degrees) of abs enc, 0 should always be horizontal!
     * @param bottomLimit lowest value the arm can achieve (soft limit) in degrees with 0 being horizontal
     * @param topLimit highest value the arm can achieve (soft limit) in degrees with 0 being horizontal
     * @throws IllegalArgumentException if each follow motor does not have an id, motor type, and inverse config
     * @throws IllegalArgumentException if the master motor does not have an id, motor type, and inverse config
     * @throws IllegalArgumentException if the any of the provided motor types are not one of the 3 expected {@link MotorConfig}
     * @throws IllegalArgumentException if the id of the backupEncoder is not part of the provided set of follow motors
     * @throws IllegalArgumentException if spinnerKPID is not three values with each being non negative
     * @throws IllegalArgumentException if spinnerKFeedForward is not three values with each being non negative
     * @throws IllegalArgumentException if gearReduction is 0
     * @throws IllegalArgumentException if maxVolts is not positive
     * @throws IllegalArgumentException if the top limit of the arm is lower or equal to the bottom limit of the arm
     */
    public ArmConfig(
        double armRelativeGearReduction, double armAbsoluteGearReduction,
        double armMaxVolts,
        int[] armFollowId, MotorConfig[] armFollowMotorType, boolean[] armFollowInversed,
        int armMasterMotorId, MotorConfig armMasterMotorType, boolean armMasterInversed,
        double[] armKPID, double armPIDTolerance, boolean armPIDContinuousInput,
        double[] armKFeedForward,
        AbsoluteEncoder armMainAbsoluteEncoder, int armMotorIDOfBackupRelativeEncoder,
        double armAbsoluteZeroOffset,
        double bottomLimit,
        double topLimit
        ) {
        
        this.armRelativeGearReduction = armRelativeGearReduction;
        this.armAbsoluteGearReduction = armAbsoluteGearReduction;
        this.armFollowId = armFollowId;
        this.armFollowMotorType = armFollowMotorType;
        this.armFollowInversed = armFollowInversed;
        this.armMasterMotorId = armMasterMotorId;
        this.armMasterMotorType = armMasterMotorType;
        this.armMasterInversed = armMasterInversed;

        this.armMainAbsoluteEncoder = armMainAbsoluteEncoder;
        this.armPIDTolerance = armPIDTolerance;
        this.armPIDContinuousInput = armPIDContinuousInput;
        this.armMaxVolts = armMaxVolts;
        this.armKPID = armKPID;
        this.armKFeedForward = armKFeedForward;
        this.armAbsoluteZeroOffset = armAbsoluteZeroOffset;
        this.bottomLimit = bottomLimit;
        this.topLimit = topLimit;
        this.armMotorOfBackupRelativeEncoderId = armMotorIDOfBackupRelativeEncoder;

        checkRequirements(armMotorIDOfBackupRelativeEncoder);

    }


    private static double defaultArmPIDTolerance = 2; //degrees

    /**
     * Create a single motor arm config with PID, no FeedForward
     * @param armMasterMotorId master id
     * @param armMasterMotorType {@link MotorConfig} of the master motor
     * @param armMasterInversed inverse config such that positive direction for the motor will result in the arm moving up
     * @param armKPID double array with 3 values, kP, kI, kD
     * @param bottomLimit lowest angle in degrees the arm can safely achieve with 0 being horizontal
     * @param topLimit highest angle in degrees the arm can safely achieve with 0 being horizontal
     * @throws IllegalArgumentException if MotorType is not one of the 3 expected {@link MotorConfig}s
     * @throws IllegalArgumentException if armKPID does not have 3 nonnegative values
     * @throws IllegalArgumentException if the bottom limit is higher or equal to the top limit
     */
    public static ArmConfig motorWithPID(int armMasterMotorId, MotorConfig armMasterMotorType, boolean armMasterInversed, double[] armKPID, double bottomLimit, double topLimit) {
        return new ArmConfig(1, 1, 14, null, null, null, armMasterMotorId, armMasterMotorType, armMasterInversed, armKPID, defaultArmPIDTolerance, false, null, null, -1, 0, bottomLimit, topLimit);
    }

    /**
     * Create a single motor arm config no PID, no FeedForward
     * @param armMasterMotorId master id
     * @param armMasterMotorType {@link MotorConfig} of the master motor
     * @param armMasterInversed inverse config such that positive direction for the motor will result in the arm moving up
     * @param bottomLimit lowest angle in degrees the arm can safely achieve with 0 being horizontal
     * @param topLimit highest angle in degrees the arm can safely achieve with 0 being horizontal
     * @throws IllegalArgumentException if MotorType is not one of the 3 expected {@link MotorConfig}s
     * @throws IllegalArgumentException if the bottom limit is higher or equal to the top limit
     */
    public static ArmConfig motorNoPID(int armMasterMotorId, MotorConfig armMasterMotorType, boolean armMasterInversed, double bottomLimit, double topLimit) {
        return motorWithPID(armMasterMotorId, armMasterMotorType, armMasterInversed, null, bottomLimit, topLimit);
    }

    /**
     * Creates a single NEO arm config with PID
     * @param armMasterMotorId master id
     * @param armMasterInversed inverse config such that positive direction for the motor will result in the arm moving up
     * @param armKPID double array with 3 values, kP, kI, kD
     * @param bottomLimit lowest angle in degrees the arm can safely achieve with 0 being horizontal
     * @param topLimit highest angle in degrees the arm can safely achieve with 0 being horizontal
     * @throws IllegalArgumentException if armKPID does not have 3 nonnegative values
     * @throws IllegalArgumentException if the bottom limit is higher or equal to the top limit
     */
    public static ArmConfig neoWithPID(int armMasterMotorId, boolean armMasterInversed, double[] armKPID, double bottomLimit, double topLimit) {
        return motorWithPID(armMasterMotorId, MotorConfig.NEO, armMasterInversed, armKPID, bottomLimit, topLimit);
    }

    /**
     * Create a single NEO arm config without PID
     * @param armMasterMotorId master id
     * @param armMasterInversed inverse config such that positive direction for the motor will result in the arm moving up
     * @param bottomLimit lowest angle in degrees the arm can safely achieve with 0 being horizontal
     * @param topLimit highest angle in degrees the arm can safely achieve with 0 being horizontal
     * @throws IllegalArgumentException if the bottom limit is higher or equal to the top limit
     */
    public static ArmConfig neoNoPID(int armMasterMotorId, boolean armMasterInversed, double bottomLimit, double topLimit) {
        return motorNoPID(armMasterMotorId, MotorConfig.NEO, armMasterInversed, bottomLimit, topLimit);
    }

    /**
     * Createa a single vortex arm config, with PID
     * @param armMasterMotorId master id
     * @param armMasterInversed inverse config such that positive direction for the motor will result in the arm moving up
     * @param armKPID double array with 3 values, kP, kI, kD
     * @param bottomLimit lowest angle in degrees the arm can safely achieve with 0 being horizontal
     * @param topLimit highest angle in degrees the arm can safely achieve with 0 being horizontal
     * @throws IllegalArgumentException if armKPID does not have 3 nonnegative values
     * @throws IllegalArgumentException if the bottom limit is higher or equal to the top limit
     */
    public static ArmConfig vortexWithPID(int armMasterMotorId, boolean armMasterInversed, double[] armKPID, double bottomLimit, double topLimit) {
        return motorWithPID(armMasterMotorId, MotorConfig.NEO_VORTEX, armMasterInversed, armKPID, bottomLimit, topLimit);
    }

    /**
     * Create a single vortex arm config, no PID
     * @param armMasterMotorId master id
     * @param armMasterInversed inverse config such that positive direction for the motor will result in the arm moving up
     * @param bottomLimit lowest angle in degrees the arm can safely achieve with 0 being horizontal
     * @param topLimit highest angle in degrees the arm can safely achieve with 0 being horizontal
     * @throws IllegalArgumentException if the bottom limit is higher or equal to the top limit
     */
    public static ArmConfig vortexNoPID(int armMasterMotorId, boolean armMasterInversed, double bottomLimit, double topLimit) {
        return motorNoPID(armMasterMotorId, MotorConfig.NEO_VORTEX, armMasterInversed, bottomLimit, topLimit);
    }

    /**
     * Check that arguments are good and set some of the parameters of the config.
     * @param armMotorOfBackupRelativeEncoderId give motor Id
     */
    private void checkRequirements(int armMotorOfBackupRelativeEncoderId) {
        matchMotors();
        checkMotorConfigs(); //Prevent future issues with new motors
        checkEncoders(armMotorOfBackupRelativeEncoderId);
        checkPID();
        checkFeedForward();
        checkGearReduction();
        checkVolts();
        checkLimits();
    }

    /**
     * Make sure that each corresponding motor id has a MotorConfig and inverse config
     */
    private void matchMotors() {
        if (armFollowId != null) {
            if (armFollowId.length == armFollowInversed.length
            && armFollowId.length == armFollowMotorType.length) {
                return;
            }
            throw new IllegalArgumentException("Motors must have matching IDs, MotorTypes and Inverse Configs");
        }
        if (armMasterMotorId != -1) {
            if (armMasterMotorType != null) {
                return;
            }
            throw new IllegalArgumentException("Master motor needs a MotorConfig");
        }
        throw new IllegalArgumentException("Master motor needs to exist");
    }

    /**
     * Check that the motor Configs are not any new configs that would not work or null.
     */
    private void checkMotorConfigs() {
        if (armFollowId != null) {
            for (MotorConfig config: armFollowMotorType) {
                if (config != MotorConfig.NEO && config != MotorConfig.NEO_VORTEX && config != MotorConfig.NEO_550) {
                    throw new IllegalArgumentException("What is this config??? If null pls change, if not null you're cooked");
                }
            }
        }
        if (armMasterMotorType != MotorConfig.NEO && armMasterMotorType != MotorConfig.NEO_VORTEX && armMasterMotorType != MotorConfig.NEO_550) {
            throw new IllegalArgumentException("What is this config??? If null pls change, if not null you're cooked");
        }
    }

    /**
     * Create {@link #armMainRelativeEncoder}, {@link #armMainAbsoluteEncoder}, and {@link #armBackupRelativeEncoder} if such are provided.
     * Will update the exists boolean for each, and throw illegal arguments if does not see the backup encoder id in the follow ids
     * @param armMotorOfBackupRelativeEncoderId
     */
    private void checkEncoders(int armMotorOfBackupRelativeEncoderId) {
        if (armMotorOfBackupRelativeEncoderId != -1) {
            if (armFollowId != null) {
                for (int i = 0; i < armFollowId.length; i++) {
                    if (armFollowId[i] == armMotorOfBackupRelativeEncoderId) {
                        return;
                    }
                }
                throw new IllegalArgumentException("Backup motor id of " + armMotorOfBackupRelativeEncoderId + " is not part of provided motors :(");
            }
            else {
                throw new IllegalArgumentException("Cannot have a backup relative encoder without follow motors");
            }
        }
        else {
            DriverStation.reportWarning("Arm does not have backup encoder, highly recommended", true);
        }

        
    }

    /**
     * Checks PID to have 3 values, updates {@link #armPIDExists} and makes sure the values are legitimate
     */
    private void checkPID() {
        if (armKPID != null) {
            if (armKPID.length == 3) {
                if (armKPID[0] < 0 || armKPID[1] < 0 || armKPID[2] < 0) {
                    throw new IllegalArgumentException("PID values have to be non negative");
                }
                armKP = armKPID[0];
                armKI = armKPID[1];
                armKD = armKPID[2];
                armPID = new PIDController(armKP, armKI, armKD);
                armPID.setTolerance(armPIDTolerance);
                if (armPIDContinuousInput) {
                    armPID.enableContinuousInput(-180, 180);
                }
                return;
            }
            throw new IllegalArgumentException("Need to have 3 values for PID");
        }
        armKP = 0;
        armKI = 0;
        armKD = 0;
        DriverStation.reportWarning("ArmPID is off", true);
    }

    /**
     * Checks FeedForward to have 4 values, updates {@link #armFeedForwardExists} and makes sure the values are legitimate
     */
    private void checkFeedForward() {
        if (armKFeedForward != null) {
            if (armKFeedForward.length == 4) {
                if (armKFeedForward[0] < 0|| armKFeedForward[2] < 0 || armKFeedForward[3] < 0) {
                    throw new IllegalArgumentException("FeedForward Values of kS, kV, and kA need to have non negative values");
                }
                armKS = armKFeedForward[0];
                armKG = armKFeedForward[1];
                armKV = armKFeedForward[2];
                armKA = armKFeedForward[3];
                armFeedForward = new ArmFeedforward(armKS, armKG, armKV, armKA);
                return;
            }
            throw new IllegalArgumentException("Need to have 4 values for Arm FeedForward: Friction, Gravity, Voltage, and Acceleration");
        }
        armKS = 0;
        armKG = 0;
        armKV = 0;
        armKA = 0;
        DriverStation.reportWarning("ArmFeedForward is off", true);
    }   

    /**
     * Makes sure gear reduction is not 0
     */
    private void checkGearReduction() {
        if (armRelativeGearReduction == 0) {
            throw new IllegalArgumentException("Gear reduction for relative encoders cannot be 0");
        }
        if (armMainAbsoluteEncoder != null && armAbsoluteGearReduction == 0) {
            throw new IllegalArgumentException("Absolute gear reduction cannot be 0 if absolute encoder exists");
        }
    }

    /**
     * Check for volt limit to be greater than 0
     */
    private void checkVolts() {
        if (armMaxVolts <= 0) {
            throw new IllegalArgumentException("armMaxVolts must be a positive number.");
        }
        return;
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