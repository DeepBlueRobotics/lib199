package org.carlmontrobotics.lib199.SimpleMechs.Arm;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;


public final class ArmConfig {
    double armRelativeGearReduction, armAbsoluteGearReduction, armPIDTolerance, armMaxVolts;
    int[] armFollowId;
    MotorConfig[] armFollowMotorType; 
    boolean[] armFollowInversed;

    int armMasterMotorId;
    MotorConfig armMasterMotorType; 
    boolean armMasterInversed;
    
    double [] armKPID, armKFeedForward;
    double armKP, armKI, armKD, armKS, armKG, armKV, armKA;
    
    AbsoluteEncoder armMainAbsoluteEncoder;
    RelativeEncoder armMainRelativeEncoder;
    RelativeEncoder armBackupRelativeEncoder;

    ArmFeedforward armFeedForward;
    PIDController armPID;
    boolean armMainAbsoluteEncoderExists, armMainRelativeEncoderExists, armBackupRelativeEncoderExists, armPIDContinuousInput, armPIDExists, armFeedForwardExists;
    //Enum<?> armStates; 

    SparkBase armMasterMotor;

    double armAbsoluteZeroOffset;
    double bottomLimit;
    double topLimit;


    

    //@param armStates provide an enum State of different positions the arm should be at, null if no state machine
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
     * @param armAbsoluteZeroOffset zeroOffset (in degrees) of abs enc, 0 should always be down!
     * @param bottomLimit lowest value the arm can achieve (soft limit)
     * @param topLimit highest value the arm can achieve (soft limit)
     */
    public ArmConfig(
        double armRelativeGearReduction, double armAbsoluteGearReduction,
        double armMaxVolts,
        int[] armFollowId, MotorConfig[] armFollowMotorType, boolean[] armFollowInversed,
        int armMasterMotorId, MotorConfig armMasterMotorType, boolean armMasterInversed,
        double[] armKPID, double armPIDTolerance, boolean armPIDContinuousInput,
        double[] armKFeedForward,
        AbsoluteEncoder armMainAbsoluteEncoder, int armMotorIDOfBackupRelativeEncoder,
        //Object armStates,
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

        checkRequirements(
            //armStates, 
            armMotorIDOfBackupRelativeEncoder);

    }


    double defaultArmPIDTolerance = 2;

    /**
     * Create a single motor arm config with PID, no FeedForward
     * @param armMasterMotorId
     * @param armMasterMotorType
     * @param armMasterInversed
     * @param armKPID
     * @param armStates
     * @param bottomLimit
     * @param topLimit
     * @return
     */
    public ArmConfig SimpleArmConfig(int armMasterMotorId, MotorConfig armMasterMotorType, boolean armMasterInversed, double[] armKPID, 
        //Object armStates, 
        double bottomLimit, double topLimit) {
        return new ArmConfig(1, 1, 14, null, null, null, armMasterMotorId, armMasterMotorType, armMasterInversed, armKPID, defaultArmPIDTolerance, false, null, null, -1, 
        //armStates, 
        0, bottomLimit, topLimit);
    }

    /**
     * Create a single motor arm config no PID, no FeedForward
     * @param armMasterMotorId
     * @param armMasterMotorType
     * @param armMasterInversed
     * @param armStates
     * @param bottomLimit
     * @param topLimit
     * @return
     */
    public ArmConfig SimpleArmConfig(int armMasterMotorId, MotorConfig armMasterMotorType, boolean armMasterInversed, 
        //Object armStates, 
        double bottomLimit, double topLimit) {
        return SimpleArmConfig(armMasterMotorId, armMasterMotorType, armMasterInversed, null, 
        //armStates, 
        bottomLimit, topLimit);
    }

    /**
     * Creates a single NEO arm config with PID
     * @param armMasterMotorId
     * @param armMasterInversed
     * @param armKPID
     * @param armStates
     * @param bottomLimit
     * @param topLimit
     * @return
     */
    public ArmConfig SimpleNeoArmConfig(int armMasterMotorId, boolean armMasterInversed, double[] armKPID, 
        //Object armStates, 
        double bottomLimit, double topLimit) {
        return SimpleArmConfig(armMasterMotorId, MotorConfig.NEO, armMasterInversed, armKPID, 
        //armStates, 
        bottomLimit, topLimit);
    }

    /**
     * Create a single NEO arm config without PID
     * @param armMasterMotorId
     * @param armMasterInversed
     * @param armStates
     * @param bottomLimit
     * @param topLimit
     * @return
     */
    public ArmConfig SimpleNeoArmConfig(int armMasterMotorId, boolean armMasterInversed, 
        //Object armStates, 
        double bottomLimit, double topLimit) {
        return SimpleArmConfig(armMasterMotorId, MotorConfig.NEO, armMasterInversed, 
        //armStates, 
        bottomLimit, topLimit);
    }

    /**
     * Createa a single vortex arm config, with PID
     * @param armMasterMotorId
     * @param armMasterInversed
     * @param armKPID
     * @param armStates
     * @param bottomLimit
     * @param topLimit
     * @return
     */
    public ArmConfig SimpleVortexArmConfig(int armMasterMotorId, boolean armMasterInversed, double[] armKPID, 
        //Object armStates, 
        double bottomLimit, double topLimit) {
        return SimpleArmConfig(armMasterMotorId, MotorConfig.NEO_VORTEX, armMasterInversed, armKPID, 
        //armStates, 
        bottomLimit, topLimit);
    }

    /**
     * Create a single vortex arm config, no PID
     * @param armMasterMotorId
     * @param armMasterInversed
     * @param armStates
     * @param bottomLimit
     * @param topLimit
     * @return
     */
    public ArmConfig SimpleVortexArmConfig(int armMasterMotorId, boolean armMasterInversed, 
        //Object armStates, 
        double bottomLimit, double topLimit) {
        return SimpleArmConfig(armMasterMotorId, MotorConfig.NEO_VORTEX, armMasterInversed, 
        //armStates, 
        bottomLimit, topLimit);
    }

    /**
     * Check that arguments are good and set some of the parameters of the config.
     * @param armStates give stateEnum
     * @param armMotorOfBackupRelativeEncoderId give motor Id
     */
    private void checkRequirements(
        //Object armStates, 
        int armMotorOfBackupRelativeEncoderId) {
        matchMotors();
        checkMotorConfigs(); //Prevent future issues with new motors
        checkEncoders(armMotorOfBackupRelativeEncoderId);
        checkPID();
        checkFeedForward();
        checkGearReduction();
        //checkEnum(armStates);
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
        for (MotorConfig config: armFollowMotorType) {
            if (config != MotorConfig.NEO && config != MotorConfig.NEO_VORTEX && config != MotorConfig.NEO_550) {
                throw new IllegalArgumentException("What is this config??? If null pls change, if not null you're cooked");
            }
        }
        if (armMasterMotorType != MotorConfig.NEO && armMasterMotorType != MotorConfig.NEO_VORTEX && armMasterMotorType != MotorConfig.NEO_550) {
            throw new IllegalArgumentException("What is this config???");
        }
    }

    /**
     * 
     * @param armMotorOfBackupRelativeEncoderId
     */
    private void checkEncoders(int armMotorOfBackupRelativeEncoderId) {
        if (armMainAbsoluteEncoder != null) {
            armMainAbsoluteEncoderExists = true;
        }
        else {
            armMainAbsoluteEncoderExists = false;
        }

        if (armMasterMotorType == MotorConfig.NEO_VORTEX) {
            SparkFlexConfig flexConfig = new SparkFlexConfig();
            this.armMasterMotor = MotorControllerFactory.createSparkFlex(armMasterMotorId);
            flexConfig.inverted(armMasterInversed)
                      .idleMode(IdleMode.kBrake)
                      .encoder.positionConversionFactor(armRelativeGearReduction * 360) // Degrees
                              .velocityConversionFactor(armRelativeGearReduction/60); //Rotations per second
            armMasterMotor.configure(flexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            armMainRelativeEncoder = armMasterMotor.getEncoder();
        }
        else {
            SparkMaxConfig maxConfig = new SparkMaxConfig();
            armMasterMotor = MotorControllerFactory.createSparkMax(armMasterMotorId, armMasterMotorType);
            maxConfig.inverted(armMasterInversed)
                      .idleMode(IdleMode.kBrake)
                      .encoder.positionConversionFactor(armRelativeGearReduction * 360) // Degrees
                              .velocityConversionFactor(armRelativeGearReduction/60); //Rotations per second
            armMasterMotor.configure(maxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            armMainRelativeEncoder = armMasterMotor.getEncoder();
        }
        armMainRelativeEncoderExists = true;
        if (armMotorOfBackupRelativeEncoderId != -1) {
            for (int i = 0; i < armFollowId.length; i++) {
                if (armFollowId[i] == armMotorOfBackupRelativeEncoderId) {
                    if (armFollowMotorType[i] == MotorConfig.NEO_VORTEX) {
                        SparkFlexConfig flexConfig = new SparkFlexConfig();
                        SparkFlex dummyFlex = MotorControllerFactory.createSparkFlex(armMasterMotorId);
                        flexConfig.encoder.positionConversionFactor(armRelativeGearReduction * 360) // Degrees
                                          .velocityConversionFactor(armRelativeGearReduction/60); //Rotations per second
                        dummyFlex.configure(flexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                        armBackupRelativeEncoder = dummyFlex.getEncoder();
                    }
                    else {
                        SparkMaxConfig maxConfig = new SparkMaxConfig();
                        SparkMax dummyMax = MotorControllerFactory.createSparkMax(armMasterMotorId, armMasterMotorType);
                        maxConfig.encoder.positionConversionFactor(armRelativeGearReduction * 360) // Degrees
                                        .velocityConversionFactor(armRelativeGearReduction/60); //Rotations per second
                        dummyMax.configure(maxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                        armBackupRelativeEncoder = dummyMax.getEncoder();
                    }
                    armMainRelativeEncoderExists = true;
                    return;
                }
            }
            throw new IllegalArgumentException("Backup motor id of " + armMotorOfBackupRelativeEncoderId + " is not part of provided motors :(");
        }
        else {
            armMainRelativeEncoderExists = false;
            System.out.println("Arm does not have backup encoder, highly recommended");
        }

        
    }

    private void checkPID() {
        if (armKPID != null) {
            if (armKPID.length == 3) {
                armKP = armKPID[0];
                armKI = armKPID[1];
                armKD = armKPID[2];
                armPID = new PIDController(armKP, armKI, armKD);
                armPID.setTolerance(armPIDTolerance);
                if (armPIDContinuousInput) {
                    armPID.enableContinuousInput(-180, 180);
                }
                armPIDExists = false;
                return;
            }
            throw new IllegalArgumentException("Need to have 3 values for PID");
        }
        armKP = 0;
        armKI = 0;
        armKD = 0;
        armPIDExists = false;
        System.out.println("ArmPID is off");
    }

    private void checkFeedForward() {
        if (armKFeedForward != null) {
            if (armKFeedForward.length == 4) {
                armKS = armKFeedForward[0];
                armKG = armKFeedForward[1];
                armKV = armKFeedForward[2];
                armKA = armKFeedForward[3];
                armFeedForward = new ArmFeedforward(armKS, armKG, armKV, armKA);
                armFeedForwardExists = true;
                return;
            }
            throw new IllegalArgumentException("Need to have 4 values for Arm FeedForward: Friction, Gravity, Voltage, and Acceleration");
        }
        armKS = 0;
        armKG = 0;
        armKV = 0;
        armFeedForwardExists = false;
        System.out.println("ArmFeedForward is off");
    }   

    private void checkGearReduction() {
        if (armRelativeGearReduction == 0) {
            throw new IllegalArgumentException("Gear reduction for relative encoders cannot be 0");
        }
        if (armMainAbsoluteEncoderExists && armAbsoluteGearReduction == 0) {
            throw new IllegalArgumentException("Absolute gear reduction cannot be 0 if absolute encoder exists");
        }
    }

    // private void checkEnum(Object armStates) {
    //     if (armStates != null) {
    //         Class<?> cls = armStates.getClass();
        
    //         if (!cls.isEnum()) {
    //             throw new IllegalArgumentException("Expected an enum");
    //         }
            
    //         java.lang.reflect.Method getValueMethod;     
    //         // Ensure a getValue() method exists
    //         try {
    //             getValueMethod = cls.getDeclaredMethod("getValue");
    //         } catch (NoSuchMethodException e) {
    //             throw new IllegalArgumentException("Enum must have a getValue() method returning the value of each state");
    //         }
        
    //         Object[] values = cls.getEnumConstants();
        
    //         for (Object constant : values) {
    //             try {
    //                 Object result = getValueMethod.invoke(constant);
        
    //                 if (!(result instanceof Number)) {
    //                     throw new IllegalArgumentException(
    //                         "getValue() must return a numeric type: " + constant);
    //                 }
        
    //                 double value = ((Number) result).doubleValue();
    //                 if (value > topLimit || value < bottomLimit) {
    //                     throw new IllegalArgumentException("Enum Position cannot be outside of the bounding limits");
    //                 }
        
    //             } catch (Exception e) {
    //                 throw new RuntimeException("Failed to read enum value", e);
    //             }
    //         }
    //         this.armStates = (Enum<?>) armStates;
    //     }
    //     else {
    //         System.out.println("Daniel is sad that you don't like states");
    //     }
    // }

    private void checkVolts() {
        if (armMaxVolts <= 0) {
            throw new IllegalArgumentException("armMaxVolts must be a positive number.");
        }
        return;
    }
    private void checkLimits() {
        if (bottomLimit >= topLimit) {
            throw new IllegalArgumentException("Top limit must be much greater than bottom limit. (Cannot be equal)");
        }
    }

    
}